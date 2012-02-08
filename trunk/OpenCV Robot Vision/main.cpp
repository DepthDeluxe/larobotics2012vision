#include <fstream>
#include <iostream>
#include <time.h>

#include <highgui.h>
#include "RobotVision.h"

using namespace std;

// edge detection variables
int lowThresh = 745;
int highThresh = 1000;
int brightThresh = 6200;
int houghThresh = 100;
bool threshChanged = false;

int currentImageView = 1;

void LoadData();
void SaveData();

void lowThresholdCallback(int value)
{
	lowThresh = value;
	threshChanged = true;
}

void highThresholdCallback(int value)
{
	highThresh = value;
	threshChanged = true;
}

void houghThesholdCallback(int value)
{
	houghThresh = value;
	threshChanged = true;
}

void main()
{
	// load file
	LoadData();

	// create instance of robot vision and initialize
	RobotVision robotVision(lowThresh, highThresh, houghThresh);
	robotVision.Initialize();

	// make windows
	cvNamedWindow("OpenCV Window", 1);
	int key = 0;

	// create trackbars for thresholding
	cvCreateTrackbar("Low Threshold", "OpenCV Window", &lowThresh, 2000, lowThresholdCallback);
	cvCreateTrackbar("High Threshold", "OpenCV Window", &highThresh, 2000, highThresholdCallback);
	cvCreateTrackbar("Hough Threshold", "OpenCV Window", &houghThresh, 250, houghThesholdCallback);

	// init fps calculator
	int frames = 0;
	time_t start = 0, end = 0;

	while (key != 'q')
	{
		// get next frame
		robotVision.GetNextFrame();

		// send threshold values to function if bool is set to true
		if (threshChanged)
		{
			robotVision.SetLowThreshold(lowThresh);
			robotVision.SetHighThreshold(highThresh);
			robotVision.SetHoughThreshold(houghThresh);
			threshChanged = false;
		}

		// do filter and transform pass
		robotVision.FilterPass();
		robotVision.TransformPass();

		// draw hough lines on filtered image
		//robotVision.DrawHoughLines();

		// draw the rectangle
		robotVision.DrawRectangle();

		// display the original image with hough lines on top of them
		if (currentImageView == 1)
			cvShowImage("OpenCV Window", robotVision.GetRawImage());
		else if (currentImageView == 2)
			cvShowImage("OpenCV Window", robotVision.GetFilteredImage());
		else if (currentImageView == 3)
			cvShowImage("OpenCV Window", robotVision.GetGrayImage());

		key = cvWaitKey(20);

		if (key == '1')
			currentImageView = 1;
		else if (key == '2')
			currentImageView = 2;
		else if (key == '3')
			currentImageView = 3;

		// update fps
		frames++;

		if (frames%15 == 0)
		{
			end = time(NULL);
			float diffTime = difftime(end, start);

			// output framerate
			cout<< "FPS: " << 1/(diffTime) << endl;

			// get start time
			start = time(NULL);
		}
	}

	// release objects
	cvDestroyWindow("OpenCV Window");
	robotVision.Dispose();

	// save file
	SaveData();
}

void SaveData()
{
	// save thresholds
	ofstream save;
	char* saveBuffer = new char[10];

	// open file for writing
	save.open("SaveData.txt", ios::out | ios::trunc);

	// write low threshold
	itoa(lowThresh, saveBuffer, 10);
	save.write(saveBuffer, strlen(saveBuffer));
	save.write("\n", 1);

	// write high threshold
	itoa(highThresh, saveBuffer, 10);
	save.write(saveBuffer, strlen(saveBuffer));
	save.write("\n", 1);

	// write hough threshold
	itoa(houghThresh, saveBuffer, 10);
	save.write(saveBuffer, strlen(saveBuffer));
	save.write("\n", 1);

	// close file
	save.close();

	delete[] saveBuffer;
}

void LoadData()
{
	ifstream load;
	char* loadBuffer = new char[10];

	load.open("SaveData.txt", ios::in);

	// if file doesn't exist, stick with default values
	if (!load.is_open())
		return;

	// get lowThreshold
	load.getline(loadBuffer, 10);
	lowThresh = atoi(loadBuffer);

	// get highThreshold
	load.getline(loadBuffer, 10);
	highThresh = atoi(loadBuffer);

	// get houghThreshold
	load.getline(loadBuffer, 10);
	houghThresh = atoi(loadBuffer);

	load.close();

	delete[] loadBuffer;
}