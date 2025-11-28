#include "VisualizationTools.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <chrono>
#include <thread>
using namespace cv;
using namespace std;

const int view_w2 = 1600;
const int view_h2 = 450;
const int view_w = 1600;
const int view_h = 900;

void drawHistogram(Mat& hist, float range[2], vector<int> peaks, string windowName) {
	CV_Assert(hist.type() == CV_32F);
	int histsize = hist.total();
	int flame_width = 1600,
		flame_height = 900;
	Mat hist_image = cv::Mat::zeros(flame_height, flame_width, CV_8UC3);
	int margin = 100;
	int width = 1600 - 2 * margin,
		height = 900 - 2 * margin;
	Mat hist_data = hist_image(cv::Rect(margin, margin, width, height));

	float bin_w = float(width) / float(histsize);
	Mat histn;
	double max_y = 0;
	minMaxLoc(hist, nullptr, &max_y);
	normalize(hist, histn, height, 0, cv::NORM_INF);
	float* data = (float*)(histn.data);
	for (int i = 0; i < histsize; i++) {
		if (data[i] == 0)
			continue;
		rectangle(hist_data, cv::Point(cvCeil(bin_w * i), cvRound(height - data[i])),
			cv::Point(cvFloor(bin_w * (i + 1)), height), Scalar(180, 180, 180), cv::FILLED);
	}
	if (!peaks.empty()) {
		for (int idx = 0; idx < peaks.size(); idx++) {
			int i = peaks[idx];
			rectangle(hist_data, cv::Point(cvCeil(bin_w * i), cvRound(height - data[i])),
				cv::Point(cvFloor(bin_w * (i + 1)), height), Scalar(0, 0, 255), cv::FILLED);
		}
	}
	// ����������
	cv::line(hist_image, cv::Point(margin, margin), cv::Point(margin, flame_height - margin - 1), Scalar(255, 255, 255)); // y��
	cv::line(hist_image, cv::Point(margin, flame_height - margin), cv::Point(flame_width - margin, flame_height - margin),
		Scalar(255, 255, 255)); // x��
	char text[10] = { 0 };
	// x��
	int textInterval = 100 / bin_w + 1;
	for (int i = 0; i <= histsize; i++) {
		double number = range[0] + i * (range[1]-range[0]) / histsize;
		if (i % textInterval == 0 || i == histsize) {
			cv::line(hist_image, cv::Point(cvRound(margin + i * bin_w), flame_height - margin + 1),
				cv::Point(cvRound(margin + i * bin_w), flame_height - margin + 15), Scalar(255, 255, 255));
			sprintf(text, "%.2f", number);
			cv::putText(hist_image, text, cv::Point(cvRound(margin + i * bin_w), flame_height - margin + 23),
				cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
		}
		else {
			cv::line(hist_image, cv::Point(cvRound(margin + i * bin_w), flame_height - margin + 1),
				cv::Point(cvRound(margin + i * bin_w), flame_height - margin + 10), Scalar(255, 255, 255));
		}
	}

	double interval = height / 8.0;
	for (int i = 1; i <= 8; i++) {
		double number = i * max_y / 8.0;
		sprintf(text, "%.2e", number);
		line(hist_image, Point(margin - 10, flame_height - margin - i * interval),
			Point(margin, flame_height - margin - i * interval), Scalar(255, 255, 255));
		putText(hist_image, text, Point(margin - 80, flame_height - margin - i * interval - 5),
			cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
	}
	cv::imshow(windowName, hist_image);
}

void drawCorners(Mat& canva, vector<Point2f>& corners) {
	CV_Assert(canva.type() == CV_8UC3);
	circle(canva, corners[0], 5, Scalar(0, 255, 255));
	for (int i = 1; i < corners.size(); i++) {
		circle(canva, corners[i], 5, Scalar(0, 255, 0));
	}
}

void dualView(cv::Mat& imgL, cv::Mat& imgR, string wdName) {
	CV_Assert(imgL.type() == imgR.type());
	int width = view_w2 / 2;
	int height = width * imgL.rows / imgL.cols;
	Mat canva(Size(view_w2, height), imgL.type());
	Mat L(canva, Rect(0, 0, width, height));
	Mat R(canva, Rect(width, 0, width, height));
	resize(imgL, L, Size(width, height));
	resize(imgR, R, Size(width, height));
	imshow(wdName, canva);
}

void showRectified(cv::Mat& imgL, cv::Mat& imgR, int lineNum, string wdName) {
	CV_Assert(imgL.type() == imgR.type());
	Mat _imgL = imgL, _imgR = imgR;
	if (_imgL.type() == CV_8U) {
		cvtColor(imgL, _imgL, COLOR_GRAY2BGR);
		cvtColor(imgR, _imgR, COLOR_GRAY2BGR);
	}
	int width = view_w2 / 2;
	int height = width * imgL.rows / imgL.cols;
	Mat canva(Size(view_w2, height), CV_8UC3);
	Mat L(canva, Rect(0, 0, width, height));
	Mat R(canva, Rect(width, 0, width, height));
	resize(imgL, L, Size(width, height));
	resize(imgR, R, Size(width, height));
	for (int i = 1; i <= lineNum; i++) {
		int d = height / (lineNum + 1);
		line(canva, Point(0, d * i), Point(view_w2, d * i), Scalar(0, 255, 0));
	}
	imshow(wdName, canva);
}

// cast Mat to [0, 255] according to the maximum and show it
Mat showMatAsImg(Mat* d_ptr, string wdName) {
	Mat canva(d_ptr->size(), CV_8U);
	double max = 0;
	minMaxLoc(*d_ptr, nullptr, &max);
	d_ptr->convertTo(canva, CV_8U, 255./max);
	namedWindow(wdName, WINDOW_NORMAL);
	imshow(wdName, canva);
	return canva;
}

// show the disparity map with an already known disparity range
Mat showDisp(Mat* d_ptr, int dispNum, string wdName) {
	Mat canva(d_ptr->size(), CV_8U);
	d_ptr->convertTo(canva, CV_8U, 255./dispNum);
	namedWindow(wdName, WINDOW_NORMAL);
	imshow(wdName, canva);
	return canva;
}

void viewCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer->initCameraParameters();
	while (!viewer->wasStopped()) {
		viewer->spinOnce();
		this_thread::sleep_for(100ms);
	}
}