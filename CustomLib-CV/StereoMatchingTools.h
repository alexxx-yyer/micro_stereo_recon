#pragma once

#include "opencv2/core.hpp"

#define CheckAndLoad(fs, node, res) \
if(fs[node].empty()) { \
	cerr << "Node " << node << " is not exsit.\n"; \
} \
fs[node] >> res;

struct CamParams {
	cv::Mat M1, M2, D1, D2, P1, P2, R1, R2, Q;
};

struct Calib2Params {
	cv::Size img_size;
	cv::Mat M1, D1, M2, D2, R, T, E, F;
};

bool loadCamParams(CamParams& params, std::string path1, std::string path2);

cv::Mat loadQMat(std::string path);

void rectifyImagePair(cv::Mat& imgL, cv::Mat& imgR, CamParams& params, cv::Mat& dstL, cv::Mat& dstR);

cv::Mat reprojectSparseTo3D(cv::Mat& disp, cv::Mat& Q);

void initPatternCoordinates(cv::Size size, float interval, std::vector<cv::Point3f>& coordinates);

void initPatternCoordinates(int w, int h, float interval, std::vector<cv::Point3f>& coordinates);