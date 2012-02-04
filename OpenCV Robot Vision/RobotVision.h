#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#define RV_DRAW_HOUGH_ON_RAW		1
#define RV_DRAW_HOUGH_ON_FILTERED	2

class RobotVision
{
	// private members
private:
	CvCapture*	camera;
	IplImage*	image;
	IplImage*	image_gray;
	IplImage*	cannyImage;

	CvMemStorage*	storage;
	CvSeq*			lineBuffer;

	int lowThreshold;
	int highThreshold;
	int houghThreshold;
public:
	RobotVision(int hi, int lo, int hou);
	void Initialize();
	void GetNextFrame();
	void FilterPass();
	void TransformPass();

	void SetLowThreshold(int value);
	void SetHighThreshold(int value);
	void SetHoughThreshold(int value);

	void DrawHoughLines();

	void Dispose();

	IplImage* GetRawImage();
	IplImage* GetFilteredImage();
	IplImage* GetGrayImage();
};