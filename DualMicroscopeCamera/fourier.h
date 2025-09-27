#pragma once

#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/types_c.h"

cv::Mat* fourierTransform(cv::Mat inputImg);

void showFT(cv::Mat inputFT);

void fftshift(cv::Mat inputImg);

cv::Mat* hpFilter(cv::Size size, int radius);