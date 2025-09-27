#include "StereoMatchingTools.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>

using namespace cv;
using namespace std;
using cvFS = FileStorage;
using cvFN = FileNode;
using cvFNIT = FileNodeIterator;

bool loadCamParams(CamParams& params, string path1, string path2) {
	if (path1.empty()) {
		cout << "Please enter the path of camera intrinsics file:\n";
		cin >> path1;
	}
	if (path2.empty()) {
		cout << "Please enter the path of camera extrinsics file:\n";
		cin >> path2;
	}
	cvFS file(path1, cvFS::READ);
	CheckAndLoad(file, "M1", params.M1);
	CheckAndLoad(file, "M2", params.M2);
	CheckAndLoad(file, "D1", params.D1);
	CheckAndLoad(file, "D2", params.D2);
	file.release();
	file.open(path2, cvFS::READ);
	CheckAndLoad(file, "P1", params.P1);
	CheckAndLoad(file, "P2", params.P2);
	CheckAndLoad(file, "R1", params.R1);
	CheckAndLoad(file, "R2", params.R2);
	CheckAndLoad(file, "Q", params.Q);
	return true;
}

void rectifyImagePair(Mat& imgL, Mat& imgR, CamParams& params, Mat& dstL, Mat& dstR) {
	Mat map[2][2];
	initUndistortRectifyMap(params.M1, params.D1, params.R1, params.P1, imgL.size(), CV_16SC2, map[0][0], map[0][1]);
	initUndistortRectifyMap(params.M2, params.D2, params.R2, params.P2, imgR.size(), CV_16SC2, map[1][0], map[1][1]);
	remap(imgL, dstL, map[0][0], map[0][1], INTER_LINEAR);
	remap(imgR, dstR, map[1][0], map[1][1], INTER_LINEAR);
}

void rectifyLeftImage(Mat& imgL, CamParams& params, Mat& dstL) {
	Mat map[2];
	initUndistortRectifyMap(params.M1, params.D1, params.R1, params.P1, imgL.size(), CV_16SC2, map[0], map[1]);
	remap(imgL, dstL, map[0], map[1], INTER_LINEAR);
}

void rectifyRightImage(Mat& imgR, CamParams& params, Mat& dstR) {
	Mat map[2];
	initUndistortRectifyMap(params.M2, params.D2, params.R2, params.P2, imgR.size(), CV_16SC2, map[0], map[1]);
	remap(imgR, dstR, map[0], map[1], INTER_LINEAR);

}

Mat reprojectSparseTo3D(Mat& disp, Mat& Q) {
	CV_Assert(disp.type() == CV_32F || disp.type() == CV_64F);
	CV_Assert(Q.size() == Size(4, 4));
	CV_Assert(disp.rows == 3);
	Mat _Q(Size(4, 4), CV_64F);
	Q.convertTo(_Q, CV_64F);
	Mat homo = Mat::ones(disp.size() + Size(0, 1), CV_64F);
	disp.convertTo(homo(Rect(0, 0, disp.cols, 3)), CV_64F);
	Mat _homo = _Q * homo;
	// 写个多线程做转换
	Mat dst(Size(3, _homo.cols), CV_64F);
	parallel_for_(Range(0, _homo.cols), [&](const Range& range)
		{
			double* ptr1 = _homo.ptr<double>(0);
			double* ptr2 = _homo.ptr<double>(1);
			double* ptr3 = _homo.ptr<double>(2);
			double* ptr4 = _homo.ptr<double>(3);
			for (int col = range.start; col < range.end; col++) {
				double* dstp = dst.ptr<double>(col);
				dstp[0] = ptr1[col] / ptr4[col];
				dstp[1] = ptr2[col] / ptr4[col];
				dstp[2] = ptr3[col] / ptr4[col];
			}
		});
	return dst;
}

Mat loadQMat(string path) {
	cvFS file(path, cvFS::READ);
	Mat res;
	CheckAndLoad(file, "Q", res);
	file.release();
	return res;
}

void initPatternCoordinates(int w, int h, float interval, vector<Point3f>& coordinates) {
	coordinates.clear();
	if (interval <= 0) {
		interval = 1.;
	}
	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; i++) {
			coordinates.push_back(Point3f(i * interval, j * interval, 0));
		}
	}
}

void initPatternCoordinates(cv::Size size, float interval, std::vector<cv::Point3f>& coordinates) {
	initPatternCoordinates(size.width, size.height, interval, coordinates);
}