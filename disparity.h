#pragma once

#ifndef _DISPARITY_H
#define _DISPARITY_H

#define _CRT_SECURE_NO_WARNINGS
#define halfImage

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

extern bool halfImageFlag;
extern string imagePlace, cameraParamter;
extern Mat imageToDisparityL, imageToDisparityR;

static Mat rectifyImageL, rectifyImageR;
static Mat disparity, disparity8U, Q;
static Mat intrinsicL, intrinsicR, distCoeffsL, distCoeffsR, R, T;
static int numDisparitiesYml, numDisparitiesHalfYml;

void getParameter();

void rectifyImageLR();

void computeDepth();

void disparitySGBM();

//void disparityBM();

void disparityGC();

void saveXYZ(const char* filename, const Mat& mat);

#endif // !_DISPARITY_H


