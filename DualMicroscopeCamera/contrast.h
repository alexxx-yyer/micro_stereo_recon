#pragma once

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/saturate.hpp"
#include "opencv2/core/fast_math.hpp"

void gammaCorrection(cv::Mat& image, double gamma);

void brightnessAdjust(cv::Mat& image, double alpha, double beta);

void histogram(cv::Mat& image, cv::Mat& hist, bool drawHist = false);