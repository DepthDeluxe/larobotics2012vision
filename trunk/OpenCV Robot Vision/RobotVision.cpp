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
	lineBuffer = cvHoughLines2(cannyImage, storage, CV_HOUGH_STANDARD, 1, CV_PI / 180, houghThreshold);
}

void RobotVision::DrawHoughLines()
{
	for (int n = 0; n < lineBuffer->total; n++)
	{
		// get a line from the buffer
		float* line = (float*)cvGetSeqElem(lineBuffer, n);

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