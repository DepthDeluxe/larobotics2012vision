#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <vector>
using namespace std;

#define RV_DRAW_HOUGH_ON_RAW		1
#define RV_DRAW_HOUGH_ON_FILTERED	2

struct RhoTheta
{
	float Rho;
	float Theta;
};

struct SlopeIntercept
{
	float Slope;
	float Intercept;
};

struct Vector2D
{
	float X;
	float Y;
};

struct Vector3D
{
	float X;
	float Y;
	float Z;
};

SlopeIntercept rvPolarToCartesian(RhoTheta);
RhoTheta rvCartesianToPolar(SlopeIntercept);

class RobotVision
{
	// private members
private:
	CvCapture*	camera;
	IplImage*	image;
	IplImage*	image_gray;
	IplImage*	cannyImage;

	CvMemStorage*		storage;
	CvSeq*				rawLineBuffer;
	vector<RhoTheta>	lineBuffer;
	vector<RhoTheta>	filteredLineBuffer;
	RhoTheta			leftSide, rightSide, topSide, bottomSide;
	Vector2D			topLeftPoint, topRightPoint, bottomLeftPoint, bottomRightPoint;
	Vector2D			rectangleCenterPoint;
	Vector3D			vectorToTarget;

	int lowThreshold;
	int highThreshold;
	int houghThreshold;
public:
	RobotVision(int hi, int lo, int hou);
	void Initialize();
	void GetNextFrame();
	void FilterPass();
	void TransformPass();
	void GetTarget();
	void CalculatePositionToTarget();

	void SetLowThreshold(int value);
	void SetHighThreshold(int value);
	void SetHoughThreshold(int value);

	void DrawHoughLines();
	void DrawRectangle();

	void Dispose();

	IplImage* GetRawImage();
	IplImage* GetFilteredImage();
	IplImage* GetGrayImage();
};