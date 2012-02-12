#include "RobotVision.h"

void rvPolarToCartesian(RhoTheta* input, SlopeIntercept* out)
{
	// condition if slope is infinity
	if (input->Theta == 0)
	{
		out->Slope = 500;
		out->Intercept = input->Rho;
	}
	else if (input->Theta > CV_PI / 2 && input->Theta < CV_PI)
	{
		out->Slope = -1/tan(input->Theta);
		out->Intercept = input->Rho * sin(input->Theta) - out->Slope * input->Rho * cos(input->Theta);
	}
	else
	{
		out->Slope = -1/tan(input->Theta);
		out->Intercept = input->Rho * sin(input->Theta) - out->Slope * input->Rho * cos(input->Theta);
	}
}

void rvCartesianToPolar(SlopeIntercept* input, RhoTheta* out)
{
	// case if slope is infinite
	if (input->Slope)
	{
		out->Rho = input->Intercept;
		out->Theta = 0;
	}
	else
	{
		out->Theta = atan(input->Slope);
		out->Rho = input->Intercept / (sin(out->Theta) - input->Slope * cos(out->Theta));
	}
}

RobotVision::RobotVision(int lo, int hi, int hou)
{
	lowThreshold = lo;
	highThreshold = hi;
	houghThreshold = hou;
}

void RobotVision::Initialize()
{
	// connect to camera
	camera = cvCaptureFromCAM(0);

	// initialize variables
	image = cvQueryFrame(camera);
	image_gray = cvCreateImage(cvGetSize(image), image->depth, 1);
	cannyImage = cvCreateImage(cvGetSize(image), image->depth, 1);

	storage = cvCreateMemStorage(0);
}

void RobotVision::GetNextFrame()
{
	// get frame
	image = cvQueryFrame(camera);

	// convert to black and white
	cvCvtColor(image, image_gray, CV_BGR2GRAY);

	// do histogram equalizing on gray image
	cvEqualizeHist(image_gray, image_gray);
}

void RobotVision::FilterPass()
{
	// do canny
	cvCanny(image_gray, cannyImage, lowThreshold, highThreshold, 3);
}

void RobotVision::TransformPass()
{
	// do standard hough transform
	rawLineBuffer = cvHoughLines2(cannyImage, storage, CV_HOUGH_STANDARD, 1, CV_PI / (180*2), houghThreshold);

	// clear lineBuffer
	lineBuffer.clear();

	RhoTheta temp;
	float* line;

	// save rawLineBuffer values into lineBuffer
	for (int n = 0; n < rawLineBuffer->total; n++)
	{
		line = (float*)cvGetSeqElem(rawLineBuffer, n);

		temp.Rho = line[0];
		temp.Theta = line[1];

		lineBuffer.push_back(temp);
	}

	// filter the lines
	GetTarget();
}

void RobotVision::GetTarget()
{
	filteredLineBuffer = lineBuffer;

	// clear filter buffer
	filteredLineBuffer.clear();

	if (lineBuffer.size() > 100 || lineBuffer.size() < 8)
		return;

	bool* isPerp = new bool[lineBuffer.size()];
	bool* isPara = new bool[lineBuffer.size()];

	// if any of the rho values are negative, set them to be +
	for (UINT n = 0; n < lineBuffer.size(); n++)
	{
		if (lineBuffer[n].Rho < 0)
		{
			lineBuffer[n].Rho = -lineBuffer[n].Rho;
			lineBuffer[n].Theta -= (float)CV_PI;
		}
	}

	// check to see if horizontal or vertical lines
	for (UINT n = 0; n < lineBuffer.size(); n++)
	{
		if ((lineBuffer[n].Theta < CV_PI / 2 + 0.3 && lineBuffer[n].Theta > CV_PI / 2 - 0.3)
			|| (lineBuffer[n].Theta < 0.3 || lineBuffer[n].Theta > CV_PI * 2 - 0.3))
		{
			filteredLineBuffer.push_back(lineBuffer[n]);
		}
	}
	
	// delete pointers related to this filtering
	delete[] isPerp;
	delete[] isPara;

	// create filter vector
	vector<RhoTheta> averageFilter = filteredLineBuffer;

	int* numAverages = new int[filteredLineBuffer.size()];
	for (UINT n = 0; n < filteredLineBuffer.size(); n++)
		numAverages[n] = 1;

	for (UINT avg = 0; avg < filteredLineBuffer.size(); avg++)
		for (UINT comp = 0; comp < filteredLineBuffer.size(); comp++)
		{
			// if their rho values are similar, average the lines
			if (abs(averageFilter[avg].Rho - filteredLineBuffer[comp].Rho) < 30
				&& abs(averageFilter[avg].Theta - filteredLineBuffer[comp].Theta) < 1)
			{
				averageFilter[avg].Rho = (averageFilter[avg].Rho * numAverages[avg] + filteredLineBuffer[comp].Rho) / (numAverages[avg]+1);
				averageFilter[avg].Theta = (averageFilter[avg].Theta * numAverages[avg] + filteredLineBuffer[comp].Theta) / (numAverages[avg]+1);

				// increase the number of averages
				numAverages[avg]++;
			}
		}

	// save modified averages back into original vector
	filteredLineBuffer = averageFilter;
	delete[] numAverages;

	// remove multiple lines
	averageFilter.clear();

	bool singlePerp = false;
	bool valueExists = false;

	for (UINT l1 = 0; l1 < filteredLineBuffer.size(); l1++)
	{
		// do perpendicular line checking
		for (UINT comp = 0; comp < filteredLineBuffer.size(); comp++)
		{
			if (abs(filteredLineBuffer[l1].Theta - filteredLineBuffer[comp].Theta) < CV_PI/2 + .25
				&& abs(filteredLineBuffer[l1].Theta - filteredLineBuffer[comp].Theta) > CV_PI/2 - .25)
			{
				singlePerp = true;
			}
		}

		// look to see if value exists
		for (UINT l2 = 0; l2 < averageFilter.size(); l2++)
		{
			if (abs(filteredLineBuffer[l1].Rho - averageFilter[l2].Rho) < 20
				&& abs(filteredLineBuffer[l1].Theta - averageFilter[l2].Theta) < 1)
			{
				valueExists = true;
			}
		}

		// if value doesn't exist, add it
		if (!valueExists && singlePerp)
		{
			averageFilter.push_back(filteredLineBuffer[l1]);
			singlePerp = false;
		}

		valueExists = false;
	}

	filteredLineBuffer = averageFilter;

	/// lines filtered ///
	// now map the rectangles

	// check to see if only 4 lines exist
	if (filteredLineBuffer.size() != 4)
		return;

	// find horizontal and vertical lines
	//
	RhoTheta horizLines[2];
	RhoTheta vertLines[2];

	int hCount = 0, vCount = 0;

	for (UINT n = 0; n < filteredLineBuffer.size(); n++)
	{
		if ((filteredLineBuffer[n].Theta < 1 || filteredLineBuffer[n].Theta > CV_PI * 2 - 1) && hCount < 2)
		{
			horizLines[hCount] = filteredLineBuffer[n];
			hCount++;
		}
		else if (vCount < 2)
		{
			vertLines[vCount] = filteredLineBuffer[n];
			vCount++;
		}
	}

	// from horizontal and vertical lines, determine which one is left and right
	if (horizLines[0].Rho > horizLines[1].Rho)
	{
		rightSide = horizLines[0];
		leftSide = horizLines[1];
	}
	if (horizLines[1].Rho > horizLines[0].Rho)
	{
		leftSide = horizLines[0];
		rightSide = horizLines[1];
	}
	if (vertLines[0].Rho > vertLines[1].Rho)
	{
		bottomSide = vertLines[0];
		topSide = vertLines[1];
	}
	if (vertLines[1].Rho > vertLines[0].Rho)
	{
		topSide = vertLines[0];
		bottomSide = vertLines[1];
	}

	// first convert to Cartesian
	SlopeIntercept lsCartesian, rsCartesian, bsCartesian, tsCartesian;
	rvPolarToCartesian(&leftSide, &lsCartesian);
	rvPolarToCartesian(&rightSide, &rsCartesian);
	rvPolarToCartesian(&topSide, &tsCartesian);
	rvPolarToCartesian(&bottomSide, &bsCartesian);

	// and find edge points
	topLeftPoint.X = (lsCartesian.Intercept - tsCartesian.Intercept) / (tsCartesian.Slope - lsCartesian.Slope);
	topLeftPoint.Y = lsCartesian.Slope * topLeftPoint.X + lsCartesian.Intercept;

	topRightPoint.X = (rsCartesian.Intercept - tsCartesian.Intercept) / (tsCartesian.Slope - rsCartesian.Slope);
	topRightPoint.Y = rsCartesian.Slope * topRightPoint.X + rsCartesian.Intercept;

	bottomLeftPoint.X = (lsCartesian.Intercept - bsCartesian.Intercept) / (bsCartesian.Slope - lsCartesian.Slope);
	bottomLeftPoint.Y = lsCartesian.Slope * bottomLeftPoint.X + lsCartesian.Intercept;

	bottomRightPoint.X = (rsCartesian.Intercept - bsCartesian.Intercept) / (bsCartesian.Slope - rsCartesian.Slope);
	bottomRightPoint.Y = rsCartesian.Slope * bottomRightPoint.X + rsCartesian.Intercept;
	
	// find the center of the rectangle by averaging the rectangle points
	rectangleCenterPoint.X = (topLeftPoint.X + topRightPoint.X) / 2;
	rectangleCenterPoint.Y = (topLeftPoint.Y + bottomLeftPoint.Y) / 2;


	// find slope and diff of slope of the lines
	float topSlope, bottomSlope, diffSlope;
	topSlope = (topRightPoint.Y - topLeftPoint.Y) / (topRightPoint.X - topLeftPoint.X);
	bottomSlope = (bottomRightPoint.Y - bottomLeftPoint.Y) / (bottomRightPoint.X - bottomLeftPoint.X);
	diffSlope = bottomSlope - topSlope;

	// find offset angle with small angle approximation
	angleOffset = RV_CAMERA_SKEW_CONST * diffSlope;

	// get the adjusted width of the rectangle for distance calculation
	float relativeWidth = (sqrt(pow(topRightPoint.X - topLeftPoint.X,2) + pow(topRightPoint.Y - topLeftPoint.Y,2))
							+ sqrt(pow(bottomRightPoint.X - bottomLeftPoint.X,2) + pow(bottomRightPoint.Y - bottomLeftPoint.Y,2))) / 2;
	float actualWidth = relativeWidth / cos(angleOffset);

	vectorToTarget.Z = RV_CAMERA_FOV_WIDTH_CONST / actualWidth;

	// get the relative size of the rectangle in the picture
	//Vector2D rectSize;
	//rectSize.X = (topRightPoint.X - topLeftPoint.X + bottomRightPoint.X - bottomLeftPoint.X) / 2;
	//rectSize.Y = (bottomRightPoint.Y - topRightPoint.Y + bottomLeftPoint.Y - topLeftPoint.Y) / 2;
	// derive distance from the frame height, becuase that doesn't really change
	//vectorToTarget.Z = (RV_CAMERA_FOV_WIDTH_CONST / rectSize.X + RV_CAMERA_FOV_HEIGHT_CONST / rectSize.Y) / 2;
}

void RobotVision::DrawRectangle()
{
	// generate points from the line
	double a = cos(leftSide.Theta);
	double b = sin(leftSide.Theta);
	double x0 = a * leftSide.Rho;
	double y0 = b * leftSide.Rho;

	// load them into CvPoint class
	CvPoint pt1, pt2;
	pt1.x = cvRound(x0 + 1000*(-b));
	pt1.y = cvRound(y0 + 1000*(a));
	pt2.x = cvRound(x0 - 1000*(-b));
	pt2.y = cvRound(y0 - 1000*(a));

	// draw left side
	cvLine( image, pt1, pt2, CV_RGB(255,0,0), 3, 8 );


	// generate points from the line
	a = cos(rightSide.Theta);
	b = sin(rightSide.Theta);
	x0 = a * rightSide.Rho;
	y0 = b * rightSide.Rho;

	// load them into CvPoint class
	pt1.x = cvRound(x0 + 1000*(-b));
	pt1.y = cvRound(y0 + 1000*(a));
	pt2.x = cvRound(x0 - 1000*(-b));
	pt2.y = cvRound(y0 - 1000*(a));

	// draw right side
	cvLine( image, pt1, pt2, CV_RGB(0,255,0), 3, 8 );


	// generate points from the line
	a = cos(topSide.Theta);
	b = sin(topSide.Theta);
	x0 = a * topSide.Rho;
	y0 = b * topSide.Rho;

	// load them into CvPoint class
	pt1.x = cvRound(x0 + 1000*(-b));
	pt1.y = cvRound(y0 + 1000*(a));
	pt2.x = cvRound(x0 - 1000*(-b));
	pt2.y = cvRound(y0 - 1000*(a));

	// draw top side
	cvLine( image, pt1, pt2, CV_RGB(0,0,255), 3, 8 );


	// generate points from the line
	a = cos(bottomSide.Theta);
	b = sin(bottomSide.Theta);
	x0 = a * bottomSide.Rho;
	y0 = b * bottomSide.Rho;

	// load them into CvPoint class
	pt1.x = cvRound(x0 + 1000*(-b));
	pt1.y = cvRound(y0 + 1000*(a));
	pt2.x = cvRound(x0 - 1000*(-b));
	pt2.y = cvRound(y0 - 1000*(a));

	// draw bottom side
	cvLine( image, pt1, pt2, CV_RGB(255,255,0), 3, 8 );

	// convert to opencv point formats
	CvPoint tl, tr, bl, br, center;
	center.x = (int)rectangleCenterPoint.X;
	center.y = (int)rectangleCenterPoint.Y;
	tl.x = (int)topLeftPoint.X;
	tl.y = (int)topLeftPoint.Y;
	tr.x = (int)topRightPoint.X;
	tr.y = (int)topRightPoint.Y;
	bl.x = (int)bottomLeftPoint.X;
	bl.y = (int)bottomLeftPoint.Y;
	br.x = (int)bottomRightPoint.X;
	br.y = (int)bottomRightPoint.Y;

	// draw points
	cvCircle(image, center, 10, CV_RGB(255,255,255));
	cvCircle(image, tl, 5, CV_RGB(255,255,255));
	cvCircle(image, tr, 5, CV_RGB(255,255,255));
	cvCircle(image, bl, 5, CV_RGB(255,255,255));
	cvCircle(image, br, 5, CV_RGB(255,255,255));

	// draw fonts telling what each point is
	char displayText[50];
	CvFont font = cvFont(1, 1);
	
	strcpy(&displayText[0], "tl");
	cvPutText(image, &displayText[0], tl, &font, CV_RGB(255,255,255));

	strcpy(&displayText[0], "tr");
	cvPutText(image, &displayText[0], tr, &font, CV_RGB(255,255,255));

	strcpy(&displayText[0], "bl");
	cvPutText(image, &displayText[0], bl, &font, CV_RGB(255,255,255));

	strcpy(&displayText[0], "br");
	cvPutText(image, &displayText[0], br, &font, CV_RGB(255,255,255));

	// draw distance on screen
	//itoa(vectorToTarget.Z, &displayText[0], 10);
	//cvPutText(image, &displayText[0], center, &font, CV_RGB(255,255,0));

	angleOffset *= 180/CV_PI;
	sprintf(&displayText[0], "%f", angleOffset);
	cvPutText(image, &displayText[0], center, &font, CV_RGB(255,255,0));
}

void RobotVision::DrawHoughLines()
{
	for (UINT n = 0; n < filteredLineBuffer.size(); n++)
	{
		// generate points from the line
		double a = cos(filteredLineBuffer[n].Theta);
		double b = sin(filteredLineBuffer[n].Theta);
		double x0 = a * filteredLineBuffer[n].Rho;
		double y0 = b * filteredLineBuffer[n].Rho;

		// load them into CvPoint class
		CvPoint pt1, pt2;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));

		// draw line
		cvLine( image, pt1, pt2, CV_RGB(255,255,0), 3, 8 );
	}
}

void RobotVision::SetLowThreshold(int value)
{
	lowThreshold = value;
}

void RobotVision::SetHighThreshold(int value)
{
	highThreshold = value;
}

void RobotVision::SetHoughThreshold(int value)
{
	houghThreshold = value;
}

IplImage* RobotVision::GetFilteredImage()
{
	return cannyImage;
}

IplImage* RobotVision::GetRawImage()
{
	return image;
}

IplImage* RobotVision::GetGrayImage()
{
	return image_gray;
}

void RobotVision::Dispose()
{
	// dispose of all opencv objects
	cvReleaseCapture(&camera);
	cvReleaseImage(&image_gray);
	cvReleaseImage(&cannyImage);
	cvReleaseMemStorage(&storage);
}