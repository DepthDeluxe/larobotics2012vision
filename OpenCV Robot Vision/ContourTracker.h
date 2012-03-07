/*
 * ContourTracker.h
 *
 *  Created on: Feb 12, 2012
 *      Author: Sammy
 */

#ifndef CONTOURTRACKER_H_
#define CONTOURTRACKER_H_

#include <iostream>
#include <vector>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"

using namespace std;
using namespace cv;

/* Brute-force method to fill in blanks in threshold images. */
void RebuildThreshold(Mat &img, int distance);

/* Find n-sided polygons and returns bounding rectangles. */
vector<Rect> FindPoly(Mat &img, unsigned int sides, unsigned int minArea, unsigned int maxSides, unsigned int maxArea);

/* Removes elements of the array which are not the specified aspect ratio. */
void TrimToAspectRatio(vector<Rect> &rect, float ratio, float errorVal);

/* Checks if given image with ROI has concentric rectangle. */
int NestedTarget(vector<Rect> &rect, Mat &img, unsigned int sides, unsigned int maxSides, unsigned int minArea, unsigned int maxArea);

#endif /* CONTOURTRACKER_H_ */
