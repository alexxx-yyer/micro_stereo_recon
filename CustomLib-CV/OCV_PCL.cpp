#include "OCV_PCL.h"

using namespace std;
using namespace cv;
using namespace pcl;

void disp2cloud(Mat* disp, Mat* Q, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	CV_Assert(disp->type() == CV_32F && Q->type() == CV_32F);
	Matx44f _Q;
	Q->convertTo(_Q, CV_32F);
	parallel_for_(Range(0, disp->rows), [&](const Range &range)
		{
			for(size_t y = range.start; y < range.end; y++) {
				float* r_ptr = disp->ptr<float>(y);
				for(size_t x = 0; x < disp->cols; x++) {
					Vec4f coordinates(x, y, r_ptr[x], 1);
					Vec4f homo = _Q * coordinates;
					homo /= homo[3];
					pcl::PointXYZ pt(homo[0], homo[1], homo[2]);
					cloud->push_back(pt);
				}
			}
		}
	);
}

void disp2cloud_rgb(cv::Mat* disp, Mat* Q, Mat* lImg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat mask) {
	CV_Assert(disp->type() == CV_32F || disp->type() == CV_64F);
	Matx44f _Q;
	Mat _disp;
	Q->convertTo(_Q, CV_32F);
	disp->convertTo(_disp, CV_32F);
	uchar* mptr = nullptr;
	for(size_t y = 0; y < disp->rows; y++) {
		float* r_ptr = _disp.ptr<float>(y);
		Vec3b* color_ptr = lImg->ptr<Vec3b>(y);
		if (!mask.empty()) {
			mptr = mask.ptr<uchar>(y);
		}
		for(size_t x = 0; x < disp->cols; x++) {
			if (mptr && *mptr == 0)
				continue;
			Vec4f coordinates(x, y, r_ptr[x], 1);
			Vec4f homo = _Q * coordinates;
			homo /= homo[3];
			pcl::PointXYZRGB pt(homo[0], homo[1], homo[2], color_ptr[x][2], color_ptr[x][1], color_ptr[x][0]);
			cloud->push_back(pt);
		}
	}
}

void xyzMat2cloud(cv::Mat* xyz, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	CV_Assert(xyz->type() == CV_64F);
	cloud->clear();
	size_t num = xyz->rows;
	for (size_t i = 0; i < num; i++) {
		double* point = xyz->ptr<double>(i);
		cloud->push_back(PointXYZ(point[0], point[1], point[2]));
	}
}

void xyzMat2cloud_rgb(cv::Mat* xyz, std::vector<cv::Vec3b>& rgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	CV_Assert(xyz->type() == CV_64F);
	cloud->clear();
	size_t num = xyz->rows;
	for (size_t i = 0; i < num; i++) {
		double* point = xyz->ptr<double>(i);
		cloud->push_back(PointXYZRGB(point[0], point[1], point[2], rgb[i][2], rgb[i][1], rgb[i][0]));
	}
}

void xyzVec2cloud(cv::Mat* xyz, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    cloud->clear();
    Vec3f* point = xyz->ptr<Vec3f>(0);
    for (size_t i = 0; i < xyz->total(); i++) {
        pcl::PointXYZ p(point[i][0], point[i][1], point[i][2]);
        cloud->push_back(p);
    }
}
