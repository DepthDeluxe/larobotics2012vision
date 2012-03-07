#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>

#include <vector>

using namespace std;
using namespace cv;

#define RV_DRAW_HOUGH_ON_RAW		1
#define RV_DRAW_HOUGH_ON_FILTERED	2

#define RV_RECTANGLE_WIDTH			24
#define RV_RECTANGLE_HEIGHT			16.5
#define RV_CAMERA_WIDTH				640
#define RV_CAMERA_HEIGHT			480
#define RV_CAMERA_FOV_WIDTH_CONST	16421.3
#define RV_CAMERA_FOV_HEIGHT_CONST	11269.3
#define RV_CAMERA_SKEW_CONST		-8.3926

#define HAWK_MIN_AREA	50
#define HAWK_AS_ERROR	0.48
#define HAWK_MAX_SIDES	15
#define HAWK_MAX_AREA	3000

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
	// functions
public:
	RobotVision(int hi, int lo, int hou);
	void Initialize();

	void GetNextFrame();
	//void DetectRectangle();
	//void LineAnalysis();

	void GetRegionOfInterest();
	void ThresholdPass();
	void TransformPass();
	void CalculatePositionToTarget();

	void SetLowThreshold(int value);
	void SetHighThreshold(int value);
	void SetHoughThreshold(int value);
	void SetBinaryThreshold(int value);

	void DrawHoughLines();
	void DrawRectangle();
	void DrawRegionOfInterest();
	void DrawImportantRectangles();

	void Dispose();

	IplImage* GetRawImage();
	IplImage* GetGrayImage();
	IplImage* GetRegionOfInterestImage();
	IplImage* GetThresholdImage();

	// private members
private:
	CvCapture*	camera;
	IplImage*	image;
	IplImage*	image_gray;
	IplImage*	regionOfInterestImage;
	IplImage*	thresholdImage;

	// variables for sammy's finder
	Mat*			matThresholdImage;
	vector<Rect>	importantRectangles;
	Rect			targetRectangle;

	CvMemStorage*		storage;
	CvSeq*				rawLineBuffer;
	vector<RhoTheta>	lineBuffer;
	vector<RhoTheta>	filteredLineBuffer;
	RhoTheta			leftSide, rightSide, topSide, bottomSide;
	Vector2D			topLeftPoint, topRightPoint, bottomLeftPoint, bottomRightPoint;
	Vector2D			rectangleCenterPoint;
	bool				trackingTarget;
	float				angleOffset;
	float				distanceToTarget;

	int lowThreshold;
	int highThreshold;
	int houghThreshold;
	int binaryThreshold;
};