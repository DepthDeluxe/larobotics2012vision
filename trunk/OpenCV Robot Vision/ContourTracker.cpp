/*
 * ContourTracker.cpp
 *
 *  Created on: Feb 12, 2012
 *      Author: Sammy
 */

#include "ContourTracker.h"
#include "assert.h"

void RebuildThreshold(Mat &img, int distance)
{
	bool prevPixel = false;
	std::vector<Point> lineMarker;

	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			/* Change image here. */
		}
		lineMarker.clear();
	}
}

vector<Rect> FindPoly(Mat &img, unsigned int sides, unsigned int maxSides, unsigned int minArea,
		unsigned int maxArea)
{
	vector<vector<Point> > contours;
	vector<Point> poly;
	vector<Rect> result;

	findContours(img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE,
			Point(0, 0));
	for (unsigned int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(contours[i], poly, arcLength(contours[i], true) * 0.02, 0);
		if (poly.size() >= sides && poly.size() <= maxSides && contourArea(poly) >= minArea
				&& contourArea(poly) <= maxArea)
			result.push_back(boundingRect(poly));
	}
	return result;
}

void TrimToAspectRatio(vector<Rect> &rect, float ratio, float errorVal)
{
	/* Aspect ratio = x:y */
	float currentRatio = 0.0;
	vector<Rect> result;
	for (unsigned int i = 0; i < rect.size(); i++)
	{
		currentRatio = (rect[i].width) / (rect[i].height);
		if (fabs(ratio - currentRatio) < errorVal)
			result.push_back(rect[i]);
	}
	rect = result;
}

int NestedTarget(vector<Rect> &rect, Mat &img, unsigned int sides, unsigned int maxSides,
		unsigned int minArea, unsigned int maxArea)
{
	vector<Rect> targets;
	Mat roi;
	for (unsigned int i = 0; i < rect.size(); i++)
	{
		roi = img(rect[i]);
		targets = FindPoly(roi, sides, maxSides, minArea, maxArea);
		if (targets.size() > 1)
			return i;
	}
	return -1;
}
