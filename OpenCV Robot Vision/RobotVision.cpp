#include "RobotVision.h"
#include "ContourTracker.h"

#include <iostream>
using namespace std;

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

void rvRotateImage(IplImage* image)
{
	Mat matImage(image);
	Point2f centerPoint(640/2, 480/2);
	matImage = getRotationMatrix2D(centerPoint, 90, 1.0);
	*image = matImage;
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
	image_threshold = cvCreateImage(cvGetSize(image), image->depth, 1);

	matThresholdImage = new Mat(image);

	storage = cvCreateMemStorage(0);

	// needs to find target before tracking
	trackingTarget = false;
}

void RobotVision::GetNextFrame()
{
	// get frame
	image = cvQueryFrame(camera);

	// get black and white frame
	cvCvtColor(image, image_gray, CV_BGR2GRAY);
}

void RobotVision::DetectRectangle()
{
	//cout<<"detecting rectangle..."<<endl;

	// filter gray image
	////////////////////
	cvThreshold(image_gray, image_threshold, binaryThreshold, 255, CV_THRESH_BINARY);

	// perform contour detection
	////////////////////////////
	CvSize size;
	size.width = 640;
	size.height = 480;

	IplImage* copiedImage = cvCreateImage(size, image->depth, 1);
	cvCopy(image_threshold, copiedImage);
	matThresholdImage = new Mat(copiedImage);

	// find the polygons
	importantRectangles = FindPoly(*matThresholdImage, 4, HAWK_MAX_SIDES, HAWK_MIN_AREA - 25, HAWK_MAX_AREA + 6000);
	TrimToAspectRatio(importantRectangles, (float)(RV_RECTANGLE_WIDTH/RV_RECTANGLE_HEIGHT), (float)HAWK_AS_ERROR);

	int targetNum = NestedTarget(importantRectangles, *matThresholdImage, 4, HAWK_MAX_SIDES, HAWK_MIN_AREA - 25, HAWK_MAX_AREA + 6000);
	if (targetNum >= 0)
	{
		targetRectangle = importantRectangles[targetNum];
		targetRectangle.x -= 10;
		targetRectangle.y -= 10;
		targetRectangle.width += 20;
		targetRectangle.height += 20;

		trackingTarget = true;
	}

	// crop the image only when it is within bounds of frame
	if (targetRectangle.x > 0 && targetRectangle.y > 0 &&
		targetRectangle.width > 0 && targetRectangle.height > 0 &&
		targetRectangle.x + targetRectangle.width < 640 &&
		targetRectangle.y + targetRectangle.height < 480 &&
		trackingTarget)
	{
		Rect cropRect(targetRectangle.x, targetRectangle.y, targetRectangle.width, targetRectangle.height);

		Mat matImage = Mat(image_gray);
		matImage = matImage(cropRect);
		image_roi = new IplImage(matImage);
	}
	else
		trackingTarget = false;

	// free memory resources
	cvReleaseImage(&copiedImage);
}

void RobotVision::LineAnalysis()
{
	// don't run anything if new target hasn't been found
	if (!trackingTarget)
		return;

	//cout<<"performing line analysis..."<<endl;

	trackingTarget = false;

	// do canny filtering
	/////////////////////
	cvCanny(image_roi, image_roi, lowThreshold, highThreshold, 3);

	// do standard hough transform and line analysis
	////////////////////////////////////////////////
	rawLineBuffer = cvHoughLines2(image_roi, storage, CV_HOUGH_STANDARD, 1, CV_PI / (180*2), houghThreshold);

	// clear lineBuffer
	lineBuffer.clear();

	RhoTheta temp;
	float* line;

	// convert CvSeq to RhoTheta format
	for (int n = 0; n < rawLineBuffer->total; n++)
	{
		line = (float*)cvGetSeqElem(rawLineBuffer, n);

		temp.Rho = line[0];
		temp.Theta = line[1];

		lineBuffer.push_back(temp);
	}

	if (lineBuffer.size() > 100 || lineBuffer.size() < 8)
		return;

	// if any of the rho values are negative, set them to be +
	for (UINT n = 0; n < lineBuffer.size(); n++)
	{
		if (lineBuffer[n].Rho < 0)
		{
			lineBuffer[n].Rho = -lineBuffer[n].Rho;
			lineBuffer[n].Theta -= (float)CV_PI;
		}
	}

	// create filter vector
	vector<RhoTheta> averageFilter = lineBuffer;

	int* numAverages = new int[lineBuffer.size()];
	for (UINT n = 0; n < lineBuffer.size(); n++)
		numAverages[n] = 1;

	for (UINT avg = 0; avg < lineBuffer.size(); avg++)
		for (UINT comp = 0; comp < lineBuffer.size(); comp++)
		{
			// if their rho values are similar, average the lines
			if (abs(averageFilter[avg].Rho - lineBuffer[comp].Rho) < 30
				&& abs(averageFilter[avg].Theta - lineBuffer[comp].Theta) < 1)
			{
				averageFilter[avg].Rho = (averageFilter[avg].Rho * numAverages[avg] + lineBuffer[comp].Rho) / (numAverages[avg]+1);
				averageFilter[avg].Theta = (averageFilter[avg].Theta * numAverages[avg] + lineBuffer[comp].Theta) / (numAverages[avg]+1);

				// increase the number of averages
				numAverages[avg]++;
			}
		}

		// save modified averages back into original vector
		lineBuffer = averageFilter;
		delete[] numAverages;

		// remove multiple lines
		averageFilter.clear();

		bool valueExists = false;

		for (UINT l1 = 0; l1 < lineBuffer.size(); l1++)
		{
			// look to see if value exists
			for (UINT l2 = 0; l2 < averageFilter.size(); l2++)
			{
				if (abs(lineBuffer[l1].Rho - averageFilter[l2].Rho) < 20
					&& abs(lineBuffer[l1].Theta - averageFilter[l2].Theta) < 1)
				{
					valueExists = true;
				}
			}

			// if value doesn't exist, add it
			if (!valueExists)
			{
				averageFilter.push_back(lineBuffer[l1]);
			}

			valueExists = false;
		}

		lineBuffer = averageFilter;

		// check to see if only 4 lines exist
		if (lineBuffer.size() != 4)
			return;

		// find horizontal and vertical lines
		//
		RhoTheta horizLines[2];
		RhoTheta vertLines[2];

		int hCount = 0, vCount = 0;

		for (UINT n = 0; n < lineBuffer.size(); n++)
		{
			if ((lineBuffer[n].Theta < 1 || lineBuffer[n].Theta > CV_PI * 2 - 1) && hCount < 2)
			{
				horizLines[hCount] = lineBuffer[n];
				hCount++;
			}
			else if (vCount < 2)
			{
				vertLines[vCount] = lineBuffer[n];
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

		if (rsCartesian.Slope != 500.0f && lsCartesian.Slope != 500.0f)
		{
			topLeftPoint.X = (lsCartesian.Intercept - tsCartesian.Intercept) / (tsCartesian.Slope - lsCartesian.Slope);
			topLeftPoint.Y = lsCartesian.Slope * topLeftPoint.X + lsCartesian.Intercept;

			topRightPoint.X = (rsCartesian.Intercept - tsCartesian.Intercept) / (tsCartesian.Slope - rsCartesian.Slope);
			topRightPoint.Y = rsCartesian.Slope * topRightPoint.X + rsCartesian.Intercept;

			bottomLeftPoint.X = (lsCartesian.Intercept - bsCartesian.Intercept) / (bsCartesian.Slope - lsCartesian.Slope);
			bottomLeftPoint.Y = lsCartesian.Slope * bottomLeftPoint.X + lsCartesian.Intercept;

			bottomRightPoint.X = (rsCartesian.Intercept - bsCartesian.Intercept) / (bsCartesian.Slope - rsCartesian.Slope);
			bottomRightPoint.Y = rsCartesian.Slope * bottomRightPoint.X + rsCartesian.Intercept;
		}
		else if (rsCartesian.Slope == 500.0f && lsCartesian.Slope != 500.0f)
		{
			topLeftPoint.X = (lsCartesian.Intercept - tsCartesian.Intercept) / (tsCartesian.Slope - lsCartesian.Slope);
			topLeftPoint.Y = lsCartesian.Slope * topLeftPoint.X + lsCartesian.Intercept;

			topRightPoint.X = rsCartesian.Intercept;
			topRightPoint.Y = tsCartesian.Slope * topRightPoint.X + tsCartesian.Intercept;

			bottomLeftPoint.X = (lsCartesian.Intercept - bsCartesian.Intercept) / (bsCartesian.Slope - lsCartesian.Slope);
			bottomLeftPoint.Y = lsCartesian.Slope * bottomLeftPoint.X + lsCartesian.Intercept;
			
			bottomRightPoint.X = rsCartesian.Intercept;
			bottomLeftPoint.Y = bsCartesian.Slope * bottomLeftPoint.X + bsCartesian.Intercept;
		}
		else if(rsCartesian.Slope != 500.0f && lsCartesian.Slope == 500.0f)
		{
			topLeftPoint.X = lsCartesian.Intercept;
			topLeftPoint.Y = tsCartesian.Slope * topLeftPoint.X + tsCartesian.Intercept;

			topRightPoint.X = (rsCartesian.Intercept - tsCartesian.Intercept) / (tsCartesian.Slope - rsCartesian.Slope);
			topRightPoint.Y = rsCartesian.Slope * topRightPoint.X + rsCartesian.Intercept;

			bottomLeftPoint.X = lsCartesian.Intercept;
			bottomLeftPoint.Y = bsCartesian.Slope * bottomLeftPoint.X + bsCartesian.Intercept;

			bottomRightPoint.X = (rsCartesian.Intercept - bsCartesian.Intercept) / (bsCartesian.Slope - rsCartesian.Slope);
			bottomRightPoint.Y = rsCartesian.Slope * bottomRightPoint.X + rsCartesian.Intercept;
		}

		// find the center of the rectangle by averaging the rectangle points
		rectangleCenterPoint.X = (topLeftPoint.X + topRightPoint.X) / 2;
		rectangleCenterPoint.Y = (topLeftPoint.Y + bottomLeftPoint.Y) / 2;


		// find slope and diff of slope of the lines
		float topSlope, bottomSlope, diffSlope;
		topSlope = (topRightPoint.Y - topLeftPoint.Y) / (topRightPoint.X - topLeftPoint.X);
		bottomSlope = (bottomRightPoint.Y - bottomLeftPoint.Y) / (bottomRightPoint.X - bottomLeftPoint.X);
		diffSlope = bottomSlope - topSlope;

		// find offset angle with small angle approximation
		angleOffset = (float)RV_CAMERA_SKEW_CONST * diffSlope;

		float relativeWidth = bottomRightPoint.X - bottomLeftPoint.X;
		float actualWidth = relativeWidth / cos(angleOffset);
		distanceToTarget = (float)RV_CAMERA_FOV_WIDTH_CONST / actualWidth;

		// free memory resources
		//cvReleaseImage(&image_roi);
}

#pragma region DRAW_FUNCTIONS
void RobotVision::DrawRectangle()
{
	//cout<<"drawing rectangle..."<<endl;

	// generate points from the line
	double a = cos(leftSide.Theta);
	double b = sin(leftSide.Theta);
	double x0 = a * leftSide.Rho;
	double y0 = b * leftSide.Rho;

	// load them into CvPoint class
	CvPoint pt1, pt2;
	pt1.x = cvRound(x0 + 1000*(-b)) + targetRectangle.x;
	pt1.y = cvRound(y0 + 1000*(a)) + targetRectangle.y;
	pt2.x = cvRound(x0 - 1000*(-b)) + targetRectangle.x;
	pt2.y = cvRound(y0 - 1000*(a)) + targetRectangle.y;

	// draw left side
	cvLine( image, pt1, pt2, CV_RGB(255,0,0), 3, 8 );


	// generate points from the line
	a = cos(rightSide.Theta);
	b = sin(rightSide.Theta);
	x0 = a * rightSide.Rho;
	y0 = b * rightSide.Rho;

	// load them into CvPoint class
	pt1.x = cvRound(x0 + 1000*(-b)) + targetRectangle.x;
	pt1.y = cvRound(y0 + 1000*(a)) + targetRectangle.y;
	pt2.x = cvRound(x0 - 1000*(-b)) + targetRectangle.x;
	pt2.y = cvRound(y0 - 1000*(a)) + targetRectangle.y;

	// draw right side
	cvLine( image, pt1, pt2, CV_RGB(0,255,0), 3, 8 );


	// generate points from the line
	a = cos(topSide.Theta);
	b = sin(topSide.Theta);
	x0 = a * topSide.Rho;
	y0 = b * topSide.Rho;

	// load them into CvPoint class
	pt1.x = cvRound(x0 + 1000*(-b)) + targetRectangle.x;
	pt1.y = cvRound(y0 + 1000*(a)) + targetRectangle.y;
	pt2.x = cvRound(x0 - 1000*(-b)) + targetRectangle.x;
	pt2.y = cvRound(y0 - 1000*(a)) + targetRectangle.y;

	// draw top side
	cvLine( image, pt1, pt2, CV_RGB(0,0,255), 3, 8 );

	// generate points from the line
	a = cos(bottomSide.Theta);
	b = sin(bottomSide.Theta);
	x0 = a * bottomSide.Rho;
	y0 = b * bottomSide.Rho;

	// load them into CvPoint class
	pt1.x = cvRound(x0 + 1000*(-b)) + targetRectangle.x;
	pt1.y = cvRound(y0 + 1000*(a)) + targetRectangle.y;
	pt2.x = cvRound(x0 - 1000*(-b)) + targetRectangle.x;
	pt2.y = cvRound(y0 - 1000*(a)) + targetRectangle.y;

	// draw bottom side
	cvLine( image, pt1, pt2, CV_RGB(255,255,0), 3, 8 );

	// convert to opencv point formats
	CvPoint tl, tr, bl, br, center;
	center.x = (int)rectangleCenterPoint.X + targetRectangle.x;
	center.y = (int)rectangleCenterPoint.Y + targetRectangle.y;
	tl.x = (int)topLeftPoint.X + targetRectangle.x;
	tl.y = (int)topLeftPoint.Y + targetRectangle.y;
	tr.x = (int)topRightPoint.X + targetRectangle.x;
	tr.y = (int)topRightPoint.Y + targetRectangle.y;
	bl.x = (int)bottomLeftPoint.X + targetRectangle.x;
	bl.y = (int)bottomLeftPoint.Y + targetRectangle.y;
	br.x = (int)bottomRightPoint.X + targetRectangle.x;
	br.y = (int)bottomRightPoint.Y + targetRectangle.y;

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

	// draw distance on screen on middle of rectangle
	sprintf(&displayText[0], "%f", distanceToTarget);
	cvPutText(image, &displayText[0], center, &font, CV_RGB(255,255,0));

	// convert offset angle to deg and save in text buffer
	float convertedOffset = (float)(angleOffset * 180/CV_PI);
	sprintf(&displayText[0], "%f", convertedOffset);

	// print offset in bottom left of screen
	CvPoint tempPt;
	tempPt.x = 5;
	tempPt.y = 470;
	cvPutText(image, &displayText[0], tempPt, &font, CV_RGB(255,255,0));
}

void RobotVision::DrawHoughLines()
{
	for (UINT n = 0; n < lineBuffer.size(); n++)
	{
		// generate points from the line
		double a = cos(lineBuffer[n].Theta);
		double b = sin(lineBuffer[n].Theta);
		double x0 = a * lineBuffer[n].Rho;
		double y0 = b * lineBuffer[n].Rho;

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

void RobotVision::DrawRegionOfInterest()
{
	CvPoint topLeft;
	topLeft.x = targetRectangle.x;
	topLeft.y = targetRectangle.y;

	CvPoint bottomRight;
	bottomRight.x = targetRectangle.x + targetRectangle.width;
	bottomRight.y = targetRectangle.y + targetRectangle.height;

	cvRectangle(image, topLeft, bottomRight, CV_RGB(0,255,0), 3);
}

void RobotVision::DrawImportantRectangles()
{
	//cout<<"drawing the important rectangles..."<<endl;

	for (UINT n = 0; n < importantRectangles.size(); n++)
	{
		CvPoint topLeft;
		topLeft.x = importantRectangles[n].x;
		topLeft.y = importantRectangles[n].y;

		CvPoint bottomRight;
		bottomRight.x = importantRectangles[n].x + importantRectangles[n].width;
		bottomRight.y = importantRectangles[n].y + importantRectangles[n].height;

		cvRectangle(image, topLeft, bottomRight, CV_RGB(0,255,0), 3);

		CvRect cropRect;
		cropRect.x = topLeft.x;
		cropRect.y = topLeft.y;
		cropRect.width = bottomRight.x - topLeft.x;
		cropRect.height = bottomRight.y - topLeft.y;
	}
}
#pragma endregion

#pragma region SET_FUNCTIONS
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

void RobotVision::SetBinaryThreshold(int value)
{
	binaryThreshold = value;
}
#pragma endregion

#pragma region RETURN_FUNCTIONS

IplImage* RobotVision::GetRawImage()
{
	return image;
}

IplImage* RobotVision::GetGrayImage()
{
	return image_gray;
}

IplImage* RobotVision::GetRegionOfInterestImage()
{
	return image_roi;
}

IplImage* RobotVision::GetThresholdImage()
{
	return image_threshold;
}
#pragma endregion

void RobotVision::Dispose()
{
	// dispose of all opencv objects
	cvReleaseCapture(&camera);
	cvReleaseImage(&image_gray);
	cvReleaseImage(&image_threshold);
	cvReleaseMemStorage(&storage);
}