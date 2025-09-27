#include "contrast.h"
#include <iostream>

using namespace cv;

void gammaCorrection(cv::Mat& image, double gamma) {
	cv::Mat lookupTable(1, 256, CV_8U);
	for (int i = 0; i < 256; ++i)
		lookupTable.data[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
	cv::LUT(image, lookupTable, image);
}

void brightnessAdjust(cv::Mat& image, double alpha, double beta) {
	CV_Assert(image.depth() == CV_8U);
	int total = image.total();
	for (int i = 0; i < total; i++)
		image.data[i] = cv::saturate_cast<uchar>(image.data[i] * alpha + beta);
}

void histogram(cv::Mat& image, cv::Mat& hist, bool drawHist) {
	CV_Assert(image.type() == CV_8U);

	int histSize = 256;
	float range[] = { 0,256 };
	const float* ranges[] = { range };
	int channels = 0;

	cv::calcHist(&image, 1, &channels, cv::Mat(), hist, 1, &histSize, ranges);
	if (drawHist) {
		int flame_width = 1600,
			flame_height = 900;
		cv::Mat hist_image = cv::Mat::zeros(flame_height, flame_width, CV_8U);
		int margin = 100;
		int width = 1600 - 2 * margin,
			height = 900 - 2 * margin;
		cv::Mat hist_data = hist_image(cv::Rect(margin, margin, width, height));

		float bin_w = width / 256.0;
		cv::normalize(hist, hist, height, 0, cv::NORM_INF);
		float* data = (float*)(hist.data);
		for (int i = 0; i < 256; i++) {
			if (data[i] == 0)
				continue;
			cv::rectangle(hist_image, cv::Point(margin + cvCeil(bin_w * i), cvRound(flame_height - margin - data[i])), 
				cv::Point(margin + cvFloor(bin_w * (i + 1)), flame_height - margin), 180, cv::FILLED);
		}
		cv::line(hist_image, cv::Point(margin - 1, margin), cv::Point(margin - 1, flame_height - margin + 1), 255);
		cv::line(hist_image, cv::Point(margin - 1, flame_height - margin + 1), cv::Point(flame_width - margin + 1, flame_height - margin + 1), 255);
		for (int i = 0; i <= 8; i++) {
			int number = i * 256 / 8;
			cv::line(hist_image, cv::Point(cvRound(margin + number * bin_w), flame_height - margin + 1), 
				cv::Point(cvRound(margin + number * bin_w), flame_height - margin + 10), 255);
			cv::putText(hist_image, std::to_string(number), cv::Point(cvRound(margin + number * bin_w), flame_height - margin + 18),
				cv::FONT_HERSHEY_SIMPLEX, 0.5, 255);
		}
		cv::imshow("hist", hist_image);
	}
}

cv::Mat bgSubstraction(cv::Mat& bg, cv::Mat& img) {
	Mat bw = abs(img - bg);
	threshold(bw, bw, 15, 255, THRESH_BINARY);
	return bw;
}