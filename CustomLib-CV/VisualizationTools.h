#pragma once

#include "pcl/io/pcd_io.h"
#include "opencv2/core.hpp"

void drawHistogram(cv::Mat& hist, float range[2], std::vector<int> peaks = {}, std::string windowName = "hist");

void drawCorners(cv::Mat& canva, std::vector<cv::Point2f>& corners);

void dualView(cv::Mat& imgL, cv::Mat& imgR, std::string wdName);

void showRectified(cv::Mat& imgL, cv::Mat& imgR, int lineNum, std::string wdName);

cv::Mat showMatAsImg(cv::Mat* d_ptr, std::string wdName);

cv::Mat showDisp(cv::Mat* d_ptr, int dispNum, std::string wdName);

void viewCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);