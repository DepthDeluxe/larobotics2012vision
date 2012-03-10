#include <fstream>
#include <iostream>

#include <opencv/highgui.h>
#include "RobotVision.h"

using namespace std;

// changeable values
// default is to change them on runtime so variables
// can change
int lowThresh = 745;
int highThresh = 1000;
int houghThresh = 100;
int binaryThresh = 197;
bool threshChanged = true;

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

void binaryThresholdCallback(int value)
{
	binaryThresh = value;
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
	cvNamedWindow("OpenCV Window");
	cvNamedWindow("Adjustment Window");
	int key = 0;

	// create trackbars for thresholding
	cvCreateTrackbar("Low Threshold", "Adjustment Window", &lowThresh, 2000, lowThresholdCallback);
	cvCreateTrackbar("High Threshold", "Adjustment Window", &highThresh, 2000, highThresholdCallback);
	cvCreateTrackbar("Hough Threshold", "Adjustment Window", &houghThresh, 250, houghThesholdCallback);
	cvCreateTrackbar("Binary Threshold", "Adjustment Window", &binaryThresh, 255, binaryThresholdCallback);

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
			robotVision.SetBinaryThreshold(binaryThresh);
			threshChanged = false;
		}

		robotVision.DetectRectangle();
		robotVision.LineAnalysis();

		// draw functions
		robotVision.DrawRectangle();
		//robotVision.DrawRegionOfInterest();
		robotVision.DrawImportantRectangles();

		// display the original image with hough lines on top of them
		if (currentImageView == 1)
			cvShowImage("OpenCV Window", robotVision.GetRawImage());
		else if (currentImageView == 2)
			cvShowImage("OpenCV Window", robotVision.GetThresholdImage());
		else if (currentImageView == 3)
			cvShowImage("OpenCV Window", robotVision.GetRegionOfInterestImage());

		key = cvWaitKey(20);

		if (key == '1')
			currentImageView = 1;
		else if (key == '2')
			currentImageView = 2;
		else if (key == '3')
			currentImageView = 3;
	}

	// release objects
	cvDestroyWindow("OpenCV Window");
	cvDestroyWindow("Adjustment Window");
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
	_itoa(lowThresh, saveBuffer, 10);
	save.write(saveBuffer, strlen(saveBuffer));
	save.write("\n", 1);

	// write high threshold
	_itoa(highThresh, saveBuffer, 10);
	save.write(saveBuffer, strlen(saveBuffer));
	save.write("\n", 1);

	// write hough threshold
	_itoa(houghThresh, saveBuffer, 10);
	save.write(saveBuffer, strlen(saveBuffer));
	save.write("\n", 1);

	// write binary threshold
	_itoa(binaryThresh, saveBuffer, 10);
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

	// get binaryThreshold
	load.getline(loadBuffer, 10);
	binaryThresh = atoi(loadBuffer);

	load.close();

	delete[] loadBuffer;
}