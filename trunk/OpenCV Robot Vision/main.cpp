#include <fstream>
#include <iostream>

#include <opencv/highgui.h>
#include "RobotVision.h"
#include "Utility.h"

#include <conio.h>

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

void main(int argc, char* argv[])
{
	bool displayWindows = true;
	bool debugOutput = false;
	char* ip;

	// process command line statements
	for (int n = 1; n < argc; n++)
	{
		if (strcmp(argv[n], "-r") == 0)
		{
			displayWindows = false;
		}

		else if (strcmp(argv[n], "-do") == 0 && argc > n+1)
		{
			debugOutput = true;
			ip = argv[n+1];
			n++;
		}

		else if (strcmp(argv[n], "?") == 0)
		{
			cout << "----- FRC 114 Robot Vision -----" << endl;
			cout << "Commands:" << endl;
			cout << "	-r: real runtime mode, windows" << endl;
			cout << "	-do: enable udp debugging output, default port is 192.168.1.14" << endl;
			cout << "		you can choose to enter another address after this" << endl;
			cout << "--------------------------------" << endl;
			return;
		}

		else
		{
			cout << "Error: you entered an invalid argument!" << endl;
			cout << "	enter ? as first argument for list of" << endl;
			cout << "	valid commands" << endl;
			return;
		}
	}

	cout<< "--- FRC 114 Robot Vision ----" << endl;
	cout << "initializing..." << endl;

	// load file
	LoadData();

	// create instance of robot vision and initialize
	cout << "loading robot vision core..." << endl;
	RobotVision robotVision(lowThresh, highThresh, houghThresh);
	robotVision.Initialize();

	// create instance of NetworkDebuggingOutput
	NetworkDebuggingOutput* output;
	if (debugOutput)
	{
			cout << "starting debugging output on port 6666" << endl;
			output = new NetworkDebuggingOutput(ip, 6666);
	}

	if (displayWindows)
	{
		// make windows
		cout << "creating view windows..." << endl;
		cvNamedWindow("OpenCV Window");
		cvNamedWindow("Adjustment Window");
		int key = 0;

		// create trackbars for thresholding
		cvCreateTrackbar("Low Threshold", "Adjustment Window", &lowThresh, 2000, lowThresholdCallback);
		cvCreateTrackbar("High Threshold", "Adjustment Window", &highThresh, 2000, highThresholdCallback);
		cvCreateTrackbar("Hough Threshold", "Adjustment Window", &houghThresh, 250, houghThesholdCallback);
		cvCreateTrackbar("Binary Threshold", "Adjustment Window", &binaryThresh, 255, binaryThresholdCallback);

		cout << "program started in debugging mode!" << endl;
		cout << "-----------------------------" << endl << endl;
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
			robotVision.DrawImportantRectangles();
			robotVision.DrawRectangle();

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

			// send to debugging output
			if (debugOutput)
				output->Send(robotVision.GetRectangleInformation());
		}
	}
	else
	{
		cout << "program started in release mode!" << endl;
		cout << "-----------------------------" << endl << endl;

		// set the thresholds
		robotVision.SetLowThreshold(lowThresh);
		robotVision.SetHighThreshold(highThresh);
		robotVision.SetHoughThreshold(houghThresh);
		robotVision.SetBinaryThreshold(binaryThresh);

		int key = 0;

		while (key != 'q')
		{
			robotVision.GetNextFrame();
			robotVision.DetectRectangle();
			robotVision.LineAnalysis();

			if (debugOutput)
			{
				output->Send(robotVision.GetRectangleInformation());
			}

			if (_kbhit() == 1)
				key = _getch();

			if (key == ' ')
			{
				cout << "Distance: " << robotVision.GetRectangleInformation().Distance << "\t";
				cout << "Offset: " << robotVision.GetRectangleInformation().AngleOffset << endl;
				key = 0;
			}
		}
	}

	cout << endl << "-----------------------------" << endl;
	cout << "closing..." << endl;

	// release objects
	if (displayWindows)
	{
		cvDestroyWindow("OpenCV Window");
		cvDestroyWindow("Adjustment Window");
	}
	robotVision.Dispose();

	// save file only if windows are shown
	if (displayWindows)
	{
		cout << "saving constants..." << endl;
		SaveData();
	}
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