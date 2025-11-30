#pragma once

#include "opencv2/core.hpp"
#include "pcl/io/pcd_io.h"


void disp2cloud(cv::Mat* disp, cv::Mat* Q, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void disp2cloud_rgb(cv::Mat* disp, cv::Mat* Q, cv::Mat* lImg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat mask = cv::Mat());

void xyzMat2cloud(cv::Mat* xyz, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void xyzMat2cloud_rgb(cv::Mat* xyz, std::vector<cv::Vec3b>& rgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

void xyzVec2cloud(cv::Mat* xyz, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);