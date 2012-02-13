#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <vector>
using namespace std;

#define RV_DRAW_HOUGH_ON_RAW		1
#define RV_DRAW_HOUGH_ON_FILTERED	2

#define RV_RECTANGLE_WIDTH			24
#define RV_RECTANGLE_HEIGHT			16.5
#define RV_CAMERA_WIDTH				640
#define RV_CAMERA_HEIGHT			480
#define RV_CAMERA_FOV_WIDTH_CONST	16421.3
#define RV_CAMERA_FOV_HEIGHT_CONST	11269.3
#define RV_CAMERA_SKEW_CONST		-8.3926

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

void rvPolarToCartesian(RhoTheta*, SlopeIntercept*);
void rvCartesianToPolar(SlopeIntercept*, RhoTheta*);

class RobotVision
{
	// private members
private:
	CvCapture*	camera;
	IplImage*	image;
	IplImage*	image_gray;
	//IplImage*	binaryFilteredImage;
	//IplImage*	contouredImage;
	IplImage*	cannyImage;

	CvMemStorage*		storage;
	CvSeq*				rawLineBuffer;
	vector<RhoTheta>	lineBuffer;
	vector<RhoTheta>	filteredLineBuffer;
	RhoTheta			leftSide, rightSide, topSide, bottomSide;
	Vector2D			topLeftPoint, topRightPoint, bottomLeftPoint, bottomRightPoint;
	Vector2D			rectangleCenterPoint;
	float				angleOffset;
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