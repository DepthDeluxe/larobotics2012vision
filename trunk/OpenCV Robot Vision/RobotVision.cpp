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
	if (lineBuffer.size() > 100 || lineBuffer.size() < 8)
		return;

	// clear filter buffer
	filteredLineBuffer.clear();

	bool* isPerp = new bool[filteredLineBuffer.size()];
	bool* isPara = new bool[filteredLineBuffer.size()];

	for (int l1 = 0; l1 < filteredLineBuffer.size(); l1++)
		for (int l2 = 0; l2 < filteredLineBuffer.size(); l2++)
		{
			if (abs(filteredLineBuffer[l1].Theta - filteredLineBuffer[l1].Theta) < 0.2)
				isPara[l1] = true;

			else if (abs(filteredLineBuffer[l1].Theta - filteredLineBuffer[l1].Theta) < CV_PI / 2 + 0.1
				&& abs(filteredLineBuffer[l1].Theta - filteredLineBuffer[l1].Theta) > CV_PI / 2 - 0.1)
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
}

void RobotVision::CalculatePositionToTarget()
{
	if (filteredLineBuffer.size() != 4)
		return;


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