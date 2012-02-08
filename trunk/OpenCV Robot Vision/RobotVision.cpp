#include "RobotVision.h"

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
	GetRectangleLines();
}

void RobotVision::GetRectangleLines()
{
	// clear filter buffer
	filteredLineBuffer.clear();

	if (lineBuffer.size() > 100 || lineBuffer.size() < 8)
		return;

	bool* isPerp = new bool[lineBuffer.size()];
	bool* isPara = new bool[lineBuffer.size()];

	// if any of the rho values are negative, set them to be +
	for (int n = 0; n < lineBuffer.size(); n++)
	{
		if (lineBuffer[n].Rho < 0)
		{
			lineBuffer[n].Rho = -lineBuffer[n].Rho;
			lineBuffer[n].Theta -= CV_PI;
		}
	}

	for (int l1 = 0; l1 < lineBuffer.size(); l1++)
		for (int l2 = 0; l2 < lineBuffer.size(); l2++)
		{
			if (abs(lineBuffer[l1].Theta - lineBuffer[l1].Theta) < 0.2)
				isPara[l1] = true;

			if (abs(lineBuffer[l1].Theta - lineBuffer[l1].Theta) < CV_PI / 2 + 0.1
				&& abs(lineBuffer[l1].Theta - lineBuffer[l1].Theta) > CV_PI / 2 - 0.1)
			{
				isPerp[l1] = true;
			}
		}

	// add if perpendicular and parallel lines exist
	for (int n = 0; n < lineBuffer.size(); n++)
	{
		if (isPerp[n] && isPara[n])
			filteredLineBuffer.push_back(lineBuffer[n]);
	}

	// delete pointers related to this filtering
	delete[] isPerp;
	delete[] isPara;

	// create filter vector
	vector<RhoTheta> averageFilter = filteredLineBuffer;

	int* numAverages = new int[filteredLineBuffer.size()];
	for (int n = 0; n < filteredLineBuffer.size(); n++)
		numAverages[n] = 1;

	for (int avg = 0; avg < filteredLineBuffer.size(); avg++)
		for (int comp = 0; comp < filteredLineBuffer.size(); comp++)
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

	for (int l1 = 0; l1 < filteredLineBuffer.size(); l1++)
	{
		// do perpendicular line checking
		for (int comp = 0; comp < filteredLineBuffer.size(); comp++)
		{
			if (abs(filteredLineBuffer[l1].Theta - filteredLineBuffer[comp].Theta) < CV_PI/2 + .25
				&& abs(filteredLineBuffer[l1].Theta - filteredLineBuffer[comp].Theta) > CV_PI/2 - .25)
			{
				singlePerp = true;
			}
		}

		// look to see if value exists
		for (int l2 = 0; l2 < averageFilter.size(); l2++)
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

	RhoTheta horizLines[2];
	RhoTheta vertLines[2];

	int hCount = 0, vCount = 0;

	for (int n = 0; n < filteredLineBuffer.size(); n++)
	{
		if (filteredLineBuffer[n].Theta < 0.2 && hCount < 2)
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

	// draw right side
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

	// draw right side
	cvLine( image, pt1, pt2, CV_RGB(255,255,0), 3, 8 );
}

void RobotVision::CalculatePositionToTarget()
{

}

void RobotVision::DrawHoughLines()
{
	for (int n = 0; n < filteredLineBuffer.size(); n++)
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