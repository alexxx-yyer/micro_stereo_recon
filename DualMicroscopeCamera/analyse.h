#pragma once

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

/****************************************  segment grid  ***********************************************/

void extractGrid(cv::Mat& input_bw, cv::Mat& output, std::vector<cv::Point>& out_points);

void extractGrid(cv::Mat& input_bw, cv::Mat& ref, cv::Mat& output);

void extractGrid(cv::Mat& input_bw, cv::Mat& output);

cv::Mat segment(cv::Mat& gray);

cv::Mat segment_ref(cv::Mat& gray);

// method2: segment image base on adaptive thresholding 
cv::Mat ATH_impl(cv::Mat& input_gray, int ws, int thresh);

cv::Mat ATH_medianBased(cv::Mat& gray, int ws);

/***************************************   locate corner  ***********************************************/


void detectCorner(cv::Mat& canva, std::vector<cv::Point> input, std::vector<cv::Point2f>& output);

void detectCorner(cv::Mat& canva, cv::Mat bw, std::vector<cv::Point2f>& output);

void bydirectionProj(cv::Mat& input, std::vector<cv::Mat> output);

std::vector<cv::Point2f> calculateCenter(cv::Mat img, std::vector<cv::Point2f> origin, int windowSize);

void detectCorner(cv::Mat& canva, cv::Mat bw, cv::Size size, std::vector<cv::Point2f>& output, double ws, bool TM_ON);

void detectCorner(cv::Mat bw, cv::Size size, std::vector<cv::Point2f>& output, double ws, bool TM_ON);

void detectCorner_fitting(cv::Mat& canva, cv::Mat bw, cv::Size size, std::vector<cv::Point2f>& output, double ws);

void detectCorner_fitting(cv::Mat bw, cv::Size size, std::vector<cv::Point2f>& output, double ws);

void detectCornerPair_fitting(cv::Mat bw1, cv::Mat bw2, cv::Size size, std::vector<cv::Point2f>& corners1, std::vector<cv::Point2f>& corners2, double ws);