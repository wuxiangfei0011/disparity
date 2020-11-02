#include "disparity.h"

void getParameter() {

	FileStorage fs(cameraParamter, FileStorage::READ);
	if (fs.isOpened()) {
		fs["intrinsicL"] >> intrinsicL;
		fs["intrinsicR"] >> intrinsicR;
		fs["distCoeffsL"] >> distCoeffsL;
		fs["distCoeffsR"] >> distCoeffsR;
		fs["R"] >> R;
		fs["T"] >> T;
		fs["numDisparities"] >> numDisparitiesYml;
		fs["numDisparitiesHalf"] >> numDisparitiesHalfYml;

		fs.release();
	}
	else cout << " can't open file 'CameraParamter.yml' " << endl;
}

void rectifyImageLR() {

	Mat RL, Rr, PL, Pr;// , Q;

	Rect validPixRoiL, validPixRoiR;
	Mat mapLx, mapLy, mapRx, mapRy;
	Size imageSize = imageToDisparityL.size();

	stereoRectify(intrinsicL, distCoeffsL, intrinsicR, distCoeffsR, imageSize,
		R, T, RL, Rr, PL, Pr, Q,
		CALIB_ZERO_DISPARITY, -1, imageSize, &validPixRoiL, &validPixRoiR);

	initUndistortRectifyMap(intrinsicL, distCoeffsL, RL, PL, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(intrinsicR, distCoeffsR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

	remap(imageToDisparityL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
	remap(imageToDisparityR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

	/***************合并左右图并绘制水平线，高乘1/2，宽不变********************************/
	Mat canvas;
	double zoom = 0.5;
	int w, h;

	w = cvRound(imageSize.width * zoom);
	h = cvRound(imageSize.height * zoom);
	canvas.create(h, w * 2, CV_8UC3);

	Mat canvasPart = canvas(Rect(w * 0, 0, w, h));
	resize(rectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
	Rect vroiL(cvRound(validPixRoiL.x * zoom), cvRound(validPixRoiL.y * zoom),
		cvRound(validPixRoiL.width * zoom), cvRound(validPixRoiL.height * zoom));
	rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);

	canvasPart = canvas(Rect(w, 0, w, h));
	resize(rectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validPixRoiR.x * zoom), cvRound(validPixRoiR.y * zoom),
		cvRound(validPixRoiR.width * zoom), cvRound(validPixRoiR.height * zoom));
	rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);

	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);

	imwrite("..\\lineVerify(disparity).png", canvas);
	cout << endl << " The horizontal-validation image have been stored in folder " << endl << endl;
	/************************************************************************************/

}

void disparitySGBM() {

	int minDisparity = 0, numDisparities = numDisparitiesYml,
		SADWindowSize = 5,
		P1 = 8 * rectifyImageL.channels() * SADWindowSize * SADWindowSize,
		P2 = 32 * rectifyImageL.channels() * SADWindowSize * SADWindowSize,
		disp12MaxDiff = -1, preFilterCap = 32, uniquenessRatio = 10,
		speckleWindowSize = 100, speckleRange = 32;


//#ifdef halfImage //将R\L图像变为 ( 1/2高*1/2宽 ) ，以加速计算
//	numDisparities = numDisparitiesHalfYml;
//
//	cout << " resize the image to ( 1/2高*1/2宽 ) to speed SGBM compution. " << endl;
//	resize(rectifyImageL, rectifyImageL, Size(rectifyImageL.cols / 2, rectifyImageL.rows / 2), 0, 0, INTER_LINEAR);
//	resize(rectifyImageR, rectifyImageR, Size(rectifyImageR.cols / 2, rectifyImageR.rows / 2), 0, 0, INTER_LINEAR);
//#endif

	if (halfImageFlag == 1) {
		numDisparities = numDisparitiesHalfYml;

		cout << " resize the image to ( 1/2高*1/2宽 ) to speed SGBM compution. " << endl;
		resize(rectifyImageL, rectifyImageL, Size(rectifyImageL.cols / 2, rectifyImageL.rows / 2), 0, 0, INTER_LINEAR);
		resize(rectifyImageR, rectifyImageR, Size(rectifyImageR.cols / 2, rectifyImageR.rows / 2), 0, 0, INTER_LINEAR);
	}

	imwrite("..\\rectifyImageL(SGBM).png", rectifyImageL);
	imwrite("..\\rectifyImageR(SGBM).png", rectifyImageR);

	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
		minDisparity, numDisparities, SADWindowSize, P1, P2,
		disp12MaxDiff, preFilterCap, uniquenessRatio, 
		speckleWindowSize, speckleRange);

	cout << " is computing disparity with SGBM...  (wait some minutes) " << endl;
	sgbm->compute(rectifyImageL, rectifyImageR, disparity); //含4个小数位的16signed int视差图，type()=3

	disparity.convertTo(disparity8U, CV_8UC1, 255 / (numDisparities * 16.0));//增加可视化效果
	//normalize(disparity, disparity8U, 0, 256, NORM_MINMAX, CV_8UC1); //与上句意义相同
	imwrite("..\\" + imagePlace + " disparity8U.png", disparity8U);

	cout << " succeed to compute disparity. 'disparity8U.png' has be storaged in folder " << endl;
}

void computeDepth() {


#if 1
	Mat worldPoint(disparity.size(), CV_32FC3); //最大测量距离与此数据类型有关
	Mat worldPoint2(disparity.size(), CV_32FC3);
	reprojectImageTo3D(disparity, worldPoint, Q, false);
	worldPoint2 = worldPoint * 16; //ReprojectTo3D求出来的是X/W,Y/W,Z/W，要除以1/16这个比例因子还不知道来源
	cout << " place setted breakpoint to get into 'image watch' " << endl; //此处设置断点，调试模式下进入image watch，查看point depth

#else //TODO:需要修改
	Mat worldPoint(disparity.size(), CV_16UC1);

	ushort* depth;
	for (int i = 0; i < disparity.rows; i++) {
		depth = worldPoint.ptr<ushort>(i);
		for (int j = 0; j < disparity.cols; j++) {
			if (!disparity.at<singed>(i, j)) continue;
			else 
				depth[j] = ushort((float)(2.7035326145486706e+03) * (1.3651264341563326e+03) / (float)disparity.at<uchar>(i, j));
		}
	}

#endif


	imwrite("..\\worldPoint.jpg", worldPoint);
	//imwrite("..\\worldPoint.tiff", worldPoint); //tiff精度更高
	//saveXYZ("..\\worldPoint.xls", worldPoint);
	
	//Mat color(dispf.size(), CV_8UC3);
	//GenerateFalseMap(disp8, color);//转成彩图
	//imshow("disparity8U", color);
}

static void saveXYZ(const char* filename, const Mat& worldPoint)
{
	const double max_z = 16.0e4;
	FILE* fp = fopen(filename, "wt");
	printf("%d %d \n", worldPoint.rows, worldPoint.cols);
	for (int y = 0; y < worldPoint.rows; y++) {
		for (int x = 0; x < worldPoint.cols; x++) {
			Vec3f point = worldPoint.at<Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
		}
	}
	fclose(fp);
}

void disparityGC() { //TODO

}