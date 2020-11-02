/*读出角点，自动比对左右图像的平齐*/
#include <iostream>
#include <fstream>
#include <string>
#include "disparity.h"

/******************************需要改参数的部分****************************************/
//#define halfImage
bool halfImageFlag = 1; //0：原图像  1：sgbm使用resize图像，加速计算 
string imagePlace = "4-15"; //标定图像路径
/**************************************************************************************/

string cameraParamter = "..\\..\\calibrate\\" + imagePlace + " CameraParamter.yml";
//Mat imageToDisparityL = imread("..\\..\\" + imagePlace + "\\tifl\\1.tif");
//Mat imageToDisparityR = imread("..\\..\\" + imagePlace + "\\tifr\\1.tif");
Mat imageToDisparityL = imread("..\\..\\" + imagePlace + "\\L\\1.png");
Mat imageToDisparityR = imread("..\\..\\" + imagePlace + "\\R\\1.png");

int main(int argc, char** argv) {

	//从calibrate工程生成的yml文件中提取参数
	getParameter();

	//将左右图像校正对齐
	rectifyImageLR();

	//计算视差图
	disparitySGBM();

	//计算空间点的深度
	computeDepth();

	system("pause");
	return 0;
}
