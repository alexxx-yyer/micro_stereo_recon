#pragma once

#include "opencv2/core.hpp"

void coarseFitting(cv::Mat& img, cv::Vec6d& params);
void coarseFittingV(cv::Mat& img, cv::Vec6d& params);

void fitting(cv::Mat& img, cv::Vec4d& params, int iter_num = 0);

void fitting(cv::Mat& img, cv::Vec6d& params, int iter_num = 0);

void fitting(cv::Mat& img, cv::Vec6d& params, double learningRate, int iter_num = 0);

// don't recommended
void fitting_newton(cv::Mat& img, cv::Vec4d& params, int iter_num = 0);

void showFunction(cv::Mat, cv::Vec4d);

void showFunction(cv::Mat, cv::Vec6d);

void showLines(cv::Mat& canva, cv::Vec6d params, double scale, cv::Scalar color);

void showFunction_lines(cv::Mat img, cv::Vec6d params, double scale);