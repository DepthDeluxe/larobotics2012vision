#include "RobotVision.h"

RobotVision::RobotVision(int hi, int low, int hou)
{
	highThreshold = hi;
	lowThreshold = low;
	houghThreshold = hou;
}

void RobotVision::Initialize()
{
	// init camera
	camera = cvCaptureFromCAM(0);

	// init all variables
	image = cvQueryFrame(camera);
	image_gray = cvCreateImage(cvGetSize(image), image->depth, 1);
	brightFilterImage = cvCreateImage(cvGetSize(image), image->depth, 1);
	cannyImage = cvCreateImage(cvGetSize(image), image->depth, 1);

	storage = cvCreateMemStorage(0);
}

void RobotVision::GetNextFrame()
{
	// get the next camera frame
	image = cvQueryFrame(camera);

	// convert to black and white
	cvCvtColor(image, image_gray, CV_BGR2GRAY);
}

void RobotVision::FilterPass()
{
	// do histogram equalizing on gray image
	cvEqualizeHist(image_gray, image_gray);

	// do brightness filter
	cvThreshold(image_gray, image_gray, brightThreshold, 255, CV_THRESH_BINARY);

	// do canny filtering
	cvCanny(image_gray, cannyImage, lowThreshold, highThreshold, 3);
}

void RobotVision::TransformPass()
{
	// perform hough transform and save it to lineBuffer
	//lineBuffer = cvHoughLines2(filtered, storage, CV_HOUGH_STANDARD, 1, CV_PI/180, houghThreshold, 0, 0);
	lineBuffer = cvHoughLines2(cannyImage, storage, CV_HOUGH_STANDARD, 1, CV_PI/(180*0.5), houghThreshold, 0, 0);
}

void RobotVision::DrawHoughLines(int imgtodrawon)
{
	bool isPerp = false;

	for (int n = 0; n < lineBuffer->total; n++)
	{
		// get individual line from line buffer
		float* line = (float*)cvGetSeqElem(lineBuffer, n);

		isPerp = true;

		// break out of iteration of loop if there is no perp
		if (!isPerp)
			continue;

		// generate points from the line
		double a = cos(line[1]);
		double b = sin(line[1]);
		double x0 = a * line[0];
		double y0 = b * line[0];

		// load them into CvPoint class
		CvPoint pt1, pt2;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));

		if (imgtodrawon == RV_DRAW_HOUGH_ON_RAW)
			cvLine( image, pt1, pt2, CV_RGB(255,255,0), 3, 8 );
		else
			cvLine( cannyImage, pt1, pt2, CV_RGB(255,255,0), 3, 8 );

		isPerp = false;
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

void RobotVision::SetBrightThreshold( int value )
{
	brightThreshold = value;
}

void RobotVision::SetHoughThreshold( int value )
{
	houghThreshold = value;
}


IplImage* RobotVision::GetRawImage()
{
	return image;
}

IplImage* RobotVision::GetFilteredImage()
{
	return cannyImage;
}

void RobotVision::Dispose()
{
	// dispose of all opencv objects
	cvReleaseCapture(&camera);
	cvReleaseImage(&image_gray);
	cvReleaseImage(&cannyImage);
	cvReleaseMemStorage(&storage);
}