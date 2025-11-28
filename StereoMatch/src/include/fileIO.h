#pragma once
#include "DirTools.h"
#include "opencv2/imgcodecs.hpp"
#include "pcl/io/pcd_io.h"
#include <string>


void saveDisp(cv::Mat& disp, std::string dir, std::string path);

void saveDisp8(cv::Mat& disp8, std::string dir, std::string path);

void xyz2cloud(cv::Mat* xyz, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void saveCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string dir, std::string path);