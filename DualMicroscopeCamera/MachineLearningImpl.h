#pragma once

#include "opencv2/core.hpp"

void fitLineRANSAC(std::vector<cv::Point>& Dataset, std::vector<cv::Point>& inlier);