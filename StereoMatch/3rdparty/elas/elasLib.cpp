// elasLib.cpp : Defines the functions for the static library.
//

#include "opencv2/core.hpp"
#include "image.h"
#include "elas.h"
#include "elasLib.h"
#include <iostream>

using namespace std;
using namespace cv;

void elas_cv(Mat* imgL, Mat* imgR, Mat* dispL, Mat* dispR) {
	CV_Assert(dispL->type() == CV_32F && dispR->type() == CV_32F);
	// check for correct size
	if (imgL->cols <= 0 || imgL->rows <= 0 || imgR->cols <= 0 || imgR->rows <= 0 ||
		imgL->cols != imgR->cols || imgL->rows != imgR->rows) {
		cout << "ERROR: Images must be of same size, but" << endl;
		cout << "       I1: " << imgL->cols << " x " << imgL->rows <<
			", I2: " << imgR->cols << " x " << imgR->rows << endl;
		return;
	}

	// get image width and height
	int32_t width = imgL->cols;
	int32_t height = imgL->rows;

	// allocate memory for disparity images
	const int32_t dims[3] = { width,height,width }; // bytes per line = width
	float* D1_data = (float*)dispL->data;
	float* D2_data = (float*)dispR->data;

	// process
	Elas::parameters param;
	param.postprocess_only_left = false;
	Elas elas(param);
	elas.process(imgL->data, imgR->data, D1_data, D2_data, dims);
	return;
}

elas_ocv::elas_ocv() {
	Elas::parameters param;
	elas = new Elas(param);
}

elas_ocv::~elas_ocv() {
	delete elas;
}

void elas_ocv::match(Mat *imgL, Mat *imgR, Mat *dispL, Mat *dispR) {
	CV_Assert(dispL->type() == CV_32F && dispR->type() == CV_32F);
	// check for correct size
	if (imgL->cols <= 0 || imgL->rows <= 0 || imgR->cols <= 0 || imgR->rows <= 0 ||
		imgL->cols != imgR->cols || imgL->rows != imgR->rows) {
		cout << "ERROR: Images must be of same size, but" << endl;
		cout << "       I1: " << imgL->cols << " x " << imgL->rows <<
			", I2: " << imgR->cols << " x " << imgR->rows << endl;
		return;
	}

	// get image width and height
	int32_t width = imgL->cols;
	int32_t height = imgL->rows;

	// allocate memory for disparity images
	const int32_t dims[3] = { width,height,width }; // bytes per line = width
	float* D1_data = (float*)dispL->data;
	float* D2_data = (float*)dispR->data;

	// process
	elas->process(imgL->data, imgR->data, D1_data, D2_data, dims);
	return;
}