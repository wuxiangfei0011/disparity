/*�����ǵ㣬�Զ��ȶ�����ͼ���ƽ��*/
#include <iostream>
#include <fstream>
#include <string>
#include "disparity.h"

/******************************��Ҫ�Ĳ����Ĳ���****************************************/
//#define halfImage
bool halfImageFlag = 1; //0��ԭͼ��  1��sgbmʹ��resizeͼ�񣬼��ټ��� 
string imagePlace = "4-15"; //�궨ͼ��·��
/**************************************************************************************/

string cameraParamter = "..\\..\\calibrate\\" + imagePlace + " CameraParamter.yml";
//Mat imageToDisparityL = imread("..\\..\\" + imagePlace + "\\tifl\\1.tif");
//Mat imageToDisparityR = imread("..\\..\\" + imagePlace + "\\tifr\\1.tif");
Mat imageToDisparityL = imread("..\\..\\" + imagePlace + "\\L\\1.png");
Mat imageToDisparityR = imread("..\\..\\" + imagePlace + "\\R\\1.png");

int main(int argc, char** argv) {

	//��calibrate�������ɵ�yml�ļ�����ȡ����
	getParameter();

	//������ͼ��У������
	rectifyImageLR();

	//�����Ӳ�ͼ
	disparitySGBM();

	//����ռ������
	computeDepth();

	system("pause");
	return 0;
}
