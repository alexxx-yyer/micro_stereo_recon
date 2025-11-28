#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "DirTools.h"
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {
	int w = 19, h = 15;
	vector<string> lImgs, rImgs;
	loadImgList_YML("E:/PictureCollection/数据采集/1129/标定/right/imglist.yml", rImgs);
	loadImgList_YML("E:/PictureCollection/数据采集/1129/标定/left/imglist.yml", lImgs);
	CV_Assert(lImgs.size() == rImgs.size());
	vector<vector<Point2f>> lCorners, rCorners;
	vector<vector<Point3f>> corners3d(1);
	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; i++) {
			corners3d[0].push_back(Point3f(i * 0.25, j * 0.25, 0));
		}
	}

	cout << "lImgs.size(): " << lImgs.size() << endl;
	int nimages = lImgs.size();
	cout << "nimages: " << nimages << endl;
	Size size;
	for (int i = 0; i < nimages; i++) {
		Mat imgL = imread(lImgs[i], IMREAD_GRAYSCALE);
		Mat imgR = imread(rImgs[i], IMREAD_GRAYSCALE);
		cout << "load: " << lImgs[i] << endl;
		cout << "load: " << rImgs[i] << endl;
		size = imgL.size();
		vector<Point2f> corners1, corners2;
		findChessboardCornersSB(imgL, Size(w, h), corners1);
		findChessboardCornersSB(imgR, Size(w, h), corners2);
		Point2f vec1 = corners1[1] - corners1[0];
		Point2f vec2 = corners2[1] - corners2[0];
		if (vec1.dot(vec2) < 0) {
			std::reverse(corners2.begin(), corners2.end());
		}
		//Mat cL, cR;
		//cvtColor(imgL, cL, COLOR_GRAY2BGR);
		//cvtColor(imgR, cR, COLOR_GRAY2BGR);
		//for (size_t k = 0; k < corners1.size(); k++) {
		//	if (k == 0) {
		//		circle(cL, corners1[k], 3, Scalar(0, 255, 255));
		//		circle(cR, corners2[k], 3, Scalar(0, 255, 255));
		//		continue;
		//	}
		//	circle(cL, corners1[k], 3, Scalar(0, 255, 0));
		//	circle(cR, corners2[k], 3, Scalar(0, 255, 0));
		//}
		//namedWindow("lcorners", WINDOW_NORMAL);
		//namedWindow("rcorners", WINDOW_NORMAL);
		//imshow("lcorners", cL);
		//imshow("rcorners", cR);
		//waitKey();
		lCorners.push_back(corners1);
		rCorners.push_back(corners2);
	}

	corners3d.resize(nimages, corners3d[0]);

	cout << "Image size: " << size << endl;
	Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = initCameraMatrix2D(corners3d, lCorners, size, 0);
	cameraMatrix[1] = initCameraMatrix2D(corners3d, rCorners, size, 0);
	Mat R, T, E, F, perError;

	cout << "Calculating reproject errors...\n";
	double rms = stereoCalibrate(corners3d, lCorners, rCorners,
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		size, R, T, E, F, perError,
		CALIB_FIX_ASPECT_RATIO +
		CALIB_ZERO_TANGENT_DIST +
		CALIB_SAME_FOCAL_LENGTH +
		CALIB_RATIONAL_MODEL +
		CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5 +
		CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 1000, 1e-6));
	cout << "done with RMS error = " << rms << endl;

	Mat R1, R2, M1, M2, Q;
	Rect* roi1 = new Rect();
	Rect* roi2 = new Rect();
	stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], size, R, T,
		R1, R2, M1, M2, Q, 0, 1, size, roi1, roi2);

	cout << "Q:\n" << Q << endl;
	printf("roi1:%d,%d,%d,%d\n", roi1->x, roi1->y, roi1->width, roi1->height);
	printf("roi2:%d,%d,%d,%d\n", roi2->x, roi2->y, roi2->width, roi2->height);
	
	Mat omap1, omap2, nmap1, nmap2;
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, M1, size, CV_32FC1, omap1, nmap1);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, M2, size, CV_32FC1, omap2, nmap2);
	for (int i = 0; i < nimages; i++) {
		Mat lImg = imread(lImgs[i]);
		Mat rImg = imread(rImgs[i]);
		Mat nl, nr;
		remap(lImg, nl, omap1, nmap1, INTER_LINEAR);
		remap(rImg, nr, omap2, nmap2, INTER_LINEAR);
		char _idx[4];
		sprintf(_idx, "_%03d", i);
		imwrite("E:/PictureCollection/数据采集/1129/rectified2/rec" + string(_idx) + "_left.png", nl(*roi1));
		imwrite("E:/PictureCollection/数据采集/1129/rectified2/rec" + string(_idx) + "_right.png", nr(*roi2));
	}
	delete roi1;
	delete roi2;
	return 0;
}