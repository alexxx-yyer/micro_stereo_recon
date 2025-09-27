#include "analyse.h"
#include "fitting.h"
#include "fourier.h"
#include <iostream>

#ifdef _DEBUG
#include <string>
#include <format>
#define CHECK_FUNC(input, res) \
	std::cout << input << std::endl << res;
#else
#define CHECK_FUNC(...)
#endif

using namespace cv;

static void showROI(Mat& roi, std::vector<Point2f> vertices, double cx, double cy, Point left_top) {
	Mat canva;
	int scale = 30;
	resize(roi, canva, Size(), scale, scale, INTER_NEAREST);
	for (int i = 0; i < 4; i++) {
		Point2f p1 = scale * (vertices[i] - Point2f(left_top));
		Point2f p2 = scale * (vertices[(i + 1) % 4] - Point2f(left_top));
		line(canva, p1, p2, Scalar(0, 255, 0), 2);
	}
	circle(canva, Point(cx * scale, cy * scale), 5, Scalar(20, 160, 220), 2);
	namedWindow("predict", WINDOW_NORMAL);
	imshow("predict", canva);
	int key = waitKey();
	if (key == 's') {
		std::string filename;
		std::cout << "Please input the file name:\n";
		std::cin >> filename;
		cv::imwrite(filename, canva);
	}
	return;
}

static Point2f subpix_Taylor(Mat input) {
	CV_Assert(input.size() == Size(3, 3) && input.type() == CV_32F);
	Point2f res;
	float* data = input.ptr<float>();
	float xp = data[3], px = data[5], yp = data[1], py = data[7], p = data[4], xyp = data[0], ypx = data[2], xpy = data[6], pyx = data[8];
	float dx, dy, dxx, dyy, dxy;
	dx = (px - xp) / 2;
	dy = (py - yp) / 2;
	dxx = xp + px - 2 * p;
	dyy = yp + py - 2 * p;
	dxy = ((pyx - xpy) / 2 - (ypx - xyp) / 2) / 2;
	Mat deriv1 = (Mat_<float>(2, 1) << dx, dy);
	Mat deriv2 = (Mat_<float>(2, 2) << dxx, dxy, dxy, dyy);
	Mat offset = -(deriv2.inv()) * deriv1;
	printf("\n[%f  %f] [%f]\n[%f  %f] [%f]\n", dxx, dxy, dx, dxy, dyy, dy);
	data = offset.ptr<float>();
	res.x = data[0];
	res.y = data[1];
	return res;
}

static Point2f subpix_centroid(Mat input) {
	CV_Assert(input.type() == CV_32F);
	int rows = input.rows;
	int cols = input.cols;
	float* data = input.ptr<float>();
	float mass = 0, xmass = 0, ymass = 0;
	for (int y = 0; y < rows; y++) {
		for (int x = 0; x < cols; x++) {
			mass = mass + *data;
			xmass = xmass + (*data) * x;
			ymass = ymass + (*data) * y;
			data++;
		}
	}
	return Point2f(xmass / mass, ymass / mass);
}

// ws:匹配窗口在原始图形中的大小 window scale
// bw:用于匹配的图片
// origin_pos:模板上待匹配点的位置
// T:透视投影的变换矩阵
// TM_ON:启用模板匹配的开关
static Point2f corner_finder(Mat bw, Point3d origin_pos, Mat T, double ws, bool TM_ON, int idx) {
	Mat corner_pos = T * Mat(origin_pos);
	double* pos_res = corner_pos.ptr<double>();
	Point2f corner_point(pos_res[0] / pos_res[2], pos_res[1] / pos_res[2]);
	// 模板匹配部分
	if (TM_ON) {
		// 读取初始模板图片
		Mat templ = imread("E:/图片收藏/数据采集/模板/cross.png", IMREAD_GRAYSCALE);
		// 获取模板四角对应的投影变换坐标
		std::vector<Point3d> window_vertices = { origin_pos + Point3d(-ws, -ws, 0), origin_pos + Point3d(ws, -ws, 0),
			origin_pos + Point3d(ws, ws, 0), origin_pos + Point3d(-ws, ws, 0) };
		std::vector<Point2f> transf_vertices;
		for (int k = 0; k < 4; k++) {
			Mat k_vertex = T * Mat(window_vertices[k]);
			double* data = k_vertex.ptr<double>();
			transf_vertices.emplace_back(data[0] / data[2], data[1] / data[2]);
		}
		// 坐标平移与窗口选取
		double min_x = DBL_MAX, min_y = DBL_MAX;
		double max_x = 0, max_y = 0;
		for (int k = 0; k < 4; k++) {
			if (transf_vertices[k].x < min_x)
				min_x = transf_vertices[k].x;
			if (transf_vertices[k].y < min_y)
				min_y = transf_vertices[k].y;
			if (transf_vertices[k].x > max_x)
				max_x = transf_vertices[k].x;
			if (transf_vertices[k].y > max_y)
				max_y = transf_vertices[k].y;
		}
		for (int k = 0; k < 4; k++) {
			transf_vertices[k] = transf_vertices[k] - Point2f(min_x, min_y);
		}
		// 对模板投影变换
		std::vector<Point2f> templ_vertices = { Point2f(0,0), Point2f(templ.cols,0),Point2f(templ.cols, templ.rows),Point2f(0,templ.rows) };
		Mat templT = getPerspectiveTransform(templ_vertices, transf_vertices);
		Mat n_templ; //new template
		warpPerspective(templ, n_templ, templT, Size(ceil(max_x - min_x), ceil(max_y - min_y)), INTER_NEAREST);
		// 确定匹配窗口
		int window_size = 5;
		Point pred_center(pos_res[0] / pos_res[2], pos_res[1] / pos_res[2]);	// 预测的交点中心
		Mat templ_center = templT * Mat(Point3d(templ.cols / 2, templ.rows / 2, 1));	// 计算模板的交点中心
		double* cdata = templ_center.ptr<double>();	// 访问指针

		Point2f direction1 = transf_vertices[1] - transf_vertices[0];
		direction1 /= norm(direction1);
		Point2f direction2 = transf_vertices[2] - transf_vertices[1];
		direction2 /= norm(direction2);
		Vec6d params(acos(direction1.x), acos(direction2.x), cdata[0] / cdata[2], cdata[1] / cdata[2], 3, 3);
		fitting(n_templ, params);
		showFunction(n_templ, params);
		waitKey();
		Point2f TC_offset(params[2], params[3]);	// 记录模板的交点中心，以模板左上角为原点

		Point pixel_offset(round(TC_offset.x), round(TC_offset.y));
		Point window_center((window_size - 1) / 2, (window_size - 1) / 2);
		Point left_top = pred_center - pixel_offset - window_center;
		Point* max_pos = new Point();
		double* max_val = new double;
		Mat match_res;
		int count = 0;
		while (1)
		{
			Mat window(bw, Rect(left_top, Size(n_templ.cols + window_size - 1, n_templ.rows + window_size - 1)));
			// 进行匹配
			matchTemplate(window, n_templ, match_res, TM_CCORR_NORMED);
			minMaxLoc(match_res, nullptr, max_val, nullptr, max_pos);

			// 测试用
#ifdef _FUNC_TEST_
			Mat canva(1080, 2160, CV_8UC3);
			Mat templ = Mat::zeros(n_templ.size(), CV_8UC3);
			Mat bg;
			cvtColor(window, bg, COLOR_GRAY2BGR);
			templ.setTo(Scalar(0, 255, 0), n_templ);
			Mat imgs(window.size() * 5, CV_8UC3);
			int w = window.size().width;
			int h = window.size().height;
			for (int i = 0; i < window_size; i++) {
				for (int j = 0; j < window_size; j++) {
					Mat painting(imgs, Rect(w * i, h * j, w, h));
					Mat bg_cpy = bg.clone();
					Mat blended(bg_cpy, Rect(i, j, templ.size().width, templ.size().height));
					add(0.5 * blended, 0.5 * templ, blended, n_templ);
					bg_cpy.copyTo(painting);
				}
			}
			resize(imgs, imgs, Size(1080, 1080), 0, 0, INTER_NEAREST);
			imgs.copyTo(Mat(canva, Rect(0, 0, 1080, 1080)));
			Mat mr = 255 * match_res;
			mr.convertTo(mr, CV_8U);
			cvtColor(mr, mr, COLOR_GRAY2BGR);
			resize(mr, mr, Size(1080, 1080), 0, 0, INTER_NEAREST);
			mr.copyTo(Mat(canva, Rect(1080, 0, 1080, 1080)));
			char image_path[100];
			sprintf_s(image_path, "E:/图片收藏/处理结果/模板匹配实验/match_res_%03d_iter%02d.png", idx, count);
			imwrite(image_path, canva);
			std::cout << '.';
#endif

			if (*max_pos == window_center || count > 10)
			{
				//Mat canva(1080, 2160, CV_8UC3);
				//Mat mr;
				//Mat templ;
				//mr = match_res * 255;
				//mr.convertTo(mr, CV_8U);
				//resize(mr, mr, Size(1080, 1080), 0, 0, INTER_NEAREST);
				//resize(n_templ, templ, Size(512, 512), 0, 0, INTER_NEAREST);
				//imwrite("C:/Users/Jay/OneDrive/work/论文专利/论文1/实验/实验1_算法全流程/templ.png", templ);
				//imwrite("C:/Users/Jay/OneDrive/work/论文专利/论文1/实验/实验1_算法全流程/match_res.png", mr);
				break;
			}
			left_top = left_top + *max_pos - window_center; // 新的匹配区域原点
			count++;
		}
		//if (*max_val < 0.2) {
		//	continue;
		//}

		// 使用二阶泰勒展开精确到亚像素(来自SIFT)
		//Mat neighbor(match_res, Rect(Point(1, 1), Size(3, 3)));
		///*Point2f center_offset(0, 0);*/
		//Point2f center_offset = subpix_Taylor(neighbor);

		// 对匹配结果使用质心法精确到亚像素
		Point2f center_offset = subpix_centroid(match_res);

		// 不做亚像素
		//Point2d center_offset = Point2d(window_size/2,window_size/2);

		//printf("offset(%f %f)\n", center_offset.x, center_offset.y);

		corner_point.x = left_top.x + TC_offset.x + center_offset.x;
		corner_point.y = left_top.y + TC_offset.y + center_offset.y;
		delete max_pos;
		delete max_val;
		return corner_point;
	}
	else {
		return corner_point;
	}
}

static Point2f corner_finder_fitting(Mat bw, Point3d origin_pos, Mat T, double ws, int idx) {
	Mat corner_pos = T * Mat(origin_pos);
	double* pos_res = corner_pos.ptr<double>();
	Point2f corner_point(pos_res[0] / pos_res[2], pos_res[1] / pos_res[2]);
	
	// 获取窗口坐标
	std::vector<Point3d> window_vertices = { origin_pos + Point3d(-ws, -ws, 0), origin_pos + Point3d(ws, -ws, 0),
		origin_pos + Point3d(ws, ws, 0), origin_pos + Point3d(-ws, ws, 0) };
	std::vector<Point2f> transf_vertices;
	for (int k = 0; k < 4; k++) {
		Mat k_vertex = T * Mat(window_vertices[k]);
		double* data = k_vertex.ptr<double>();
		transf_vertices.emplace_back(data[0] / data[2], data[1] / data[2]);
	}
	// 坐标平移与窗口选取
	double min_x = DBL_MAX, min_y = DBL_MAX;
	double max_x = 0, max_y = 0;
	for (int k = 0; k < 4; k++) {
		if (transf_vertices[k].x < min_x)
			min_x = transf_vertices[k].x;
		if (transf_vertices[k].y < min_y)
			min_y = transf_vertices[k].y;
		if (transf_vertices[k].x > max_x)
			max_x = transf_vertices[k].x;
		if (transf_vertices[k].y > max_y)
			max_y = transf_vertices[k].y;
	}

	// 确定匹配窗口
	Point left_top(floor(min_x), floor(min_y));
	Point right_bottom(ceil(max_x), ceil(max_y));
	Mat dataset(bw, Rect(left_top, right_bottom));
	// 确定初始参数
	double cx = pos_res[0] / pos_res[2];	// 预测的交点中心
	double cy = pos_res[1] / pos_res[2];
	cx -= left_top.x;
	cy -= left_top.y;
	Point2f direction1 = transf_vertices[1] - transf_vertices[0];
	direction1 /= norm(direction1);
	Point2f direction2 = transf_vertices[2] - transf_vertices[1];
	direction2 /= norm(direction2);
	Vec6d params(acos(direction1.x), acos(direction2.x), cx, cy, 3, 3);
	coarseFitting(dataset, params);
	fitting(dataset, params, 10000);
#ifdef CORNERDETECTOR_EXPORTS
#ifdef _DEBUG
	std::cout << format("params: %f %f %f %f %f %f\n", params[0], params[1], params[2], params[3], params[4], params[5]);
#endif
#endif

	corner_point.x = params[2] + left_top.x;
	corner_point.y = params[3] + left_top.y;
	return corner_point;
}

static Point2f corner_finder_fitting(Mat& canva, Mat bw, Point3d origin_pos, Mat T, double ws, int idx) {
	int showN = 42;
	Mat corner_pos = T * Mat(origin_pos);
	double* pos_res = corner_pos.ptr<double>();
	Point2f corner_point(pos_res[0] / pos_res[2], pos_res[1] / pos_res[2]);
	//circle(canva, corner_point, 5, Scalar(0, 255, 0));

	// 获取窗口坐标
	std::vector<Point3d> window_vertices = { origin_pos + Point3d(-ws, -ws, 0), origin_pos + Point3d(ws, -ws, 0),
		origin_pos + Point3d(ws, ws, 0), origin_pos + Point3d(-ws, ws, 0) };
	std::vector<Point2f> transf_vertices;
	for (int k = 0; k < 4; k++) {
		Mat k_vertex = T * Mat(window_vertices[k]);
		double* data = k_vertex.ptr<double>();
		transf_vertices.emplace_back(data[0] / data[2], data[1] / data[2]);
	}
	// 坐标平移与窗口选取
	double min_x = DBL_MAX, min_y = DBL_MAX;
	double max_x = 0, max_y = 0;
	for (int k = 0; k < 4; k++) {
		if (transf_vertices[k].x < min_x)
			min_x = transf_vertices[k].x;
		if (transf_vertices[k].y < min_y)
			min_y = transf_vertices[k].y;
		if (transf_vertices[k].x > max_x)
			max_x = transf_vertices[k].x;
		if (transf_vertices[k].y > max_y)
			max_y = transf_vertices[k].y;
	}

	// 确定匹配窗口
	Point left_top(floor(min_x), floor(min_y));
	Point right_bottom(ceil(max_x), ceil(max_y));
	Mat dataset(bw, Rect(left_top, right_bottom));
	// 确定初始参数
	double cx = pos_res[0] / pos_res[2];	// 预测的交点中心
	double cy = pos_res[1] / pos_res[2];
	cx -= left_top.x;
	cy -= left_top.y;
	if (idx == showN) {
		Mat showI(canva, Rect(left_top, right_bottom));
		showROI(showI, transf_vertices, cx, cy, left_top);
	}
	Point2f direction1 = transf_vertices[1] - transf_vertices[0];
	direction1 /= norm(direction1);
	Point2f direction2 = transf_vertices[2] - transf_vertices[1];
	direction2 /= norm(direction2);
	Vec6d params(acos(direction1.x), acos(direction2.x), cx, cy, 3, 3);

	int scale = 20;
	if (idx == showN) {
		cv::cvtColor(dataset, canva, COLOR_GRAY2BGR);
		cv::resize(canva, canva, Size(), scale, scale, INTER_NEAREST);
		imshow("dataset", canva);
		int key = waitKey();
		if (key == 's') {
			printf("Please enter the path to save image:\n");
			std::string p;
			std::cin >> p;
			cv::imwrite(p, canva);
		}
		//showLines(canva, params, scale, Scalar(0, 255, 255));
		imshow("predicted", canva);
		key = waitKey();
		if (key == 's') {
			printf("Please enter the path to save image:\n");
			std::string p;
			std::cin >> p;
			cv::imwrite(p, canva);
		}
		coarseFittingV(dataset, params);
		showFunction_lines(dataset, params, 40);
		showLines(canva, params, scale, Scalar(255, 0, 0));
		fitting(dataset, params, 10000);
		showFunction_lines(dataset, params, 40);
		showLines(canva, params, scale, Scalar(0, 255, 0));
		imshow("compare c&f", canva);
		key = waitKey();
		if (key == 's') {
			printf("Please enter the path to save image:\n");
			std::string p;
			std::cin >> p;
			cv::imwrite(p, canva);
		}
	}
	else {
		coarseFitting(dataset, params);
		fitting(dataset, params, 10000);
	}
	
	//fitting(dataset, params, 4.51e-2, 10000);
#ifdef CORNERDETECTOR_EXPORTS
#ifdef _DEBUG
	std::cout << format("params: %f %f %f %f %f %f\n", params[0], params[1], params[2], params[3], params[4], params[5]);
#endif
#endif

	corner_point.x = params[2] + left_top.x;
	corner_point.y = params[3] + left_top.y;
	return corner_point;
}

static void model_fitting(Mat& bw, Mat T, double ws, std::vector<Point3d>& initCorners, std::vector<Point2f>& output) {
	int count = 0;
	for (; count < initCorners.size(); count++) {
		Point2f corner_point = corner_finder_fitting(bw, initCorners[count], T, ws, count);
#ifdef NDEBUG
		std::cout << "..." << count + 1 << "/" << initCorners.size() << "\r";
#endif
		output.push_back(corner_point);
	}
	std::cout << '\n';
}

// 质心法计算角点坐标
std::vector<Point2f> calculateCenter(Mat img, std::vector<Point2f> origin, int windowSize) {
	CV_Assert(img.depth() == CV_8U);
	std::vector<Point2f> output;
	int delta = windowSize / 2;
	for (Point2f current : origin) {
		int counter = 0;
		while(1)
		{
			// 根据原始输入点构建窗口		
			Range rowRange(std::max(int(current.y) - delta, 0), std::min(int(current.y) + delta + 1, img.rows)),
				colRange(std::max(int(current.x) - delta, 0), std::min(int(current.x) + delta + 1, img.cols));
			Mat window(img, rowRange, colRange);
			// 在窗口内计算质心
			int Mass = 0, xMass = 0, yMass = 0;
			for (int i = 0; i < window.rows; i++) {
				uchar* I = window.ptr(i);
				for (int j = 0; j < window.cols; j++) {
					int x = colRange.start + j,
						y = rowRange.start + i;
					Mass = Mass + (*I);
					xMass = xMass + (*I) * x;
					yMass = yMass + (*I) * y;
					I++;
				}
			}
			Point2f center(xMass / Mass, yMass / Mass);
			Point2f dv = center - current; // 位移矢量
			float distance = dv.ddot(dv);
			current = center;
			counter++;
			// 将计算出的质心与原始点比较，如果变化小于一定值或者达到迭代次数限制则记录
			if (distance <= 1e-5 || counter >= 10)
				break;
		}
		output.push_back(current);
	}
	return output;
}

// 方案2：
// step1: 透视投影变换获取初始角点位置
// step2: 计算质心获取准确位置
void detectCorner(cv::Mat& canva, cv::Mat bw, std::vector<cv::Point2f>& output) {
	// 1.找出近似四边形
	std::vector<std::vector<Point>> contours;
	findContours(bw, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	std::vector<Point> contour = contours[0];
	bool closed = true;
	approxPolyDP(contour, contour, 40, closed);
	for (int i = 0; i < contour.size(); i++) {
		line(canva, contour[i], contour[(i+1)%contour.size()], Scalar(0, 255, 0));
	}
	// 2.计算交点
	//  2.1 原始坐标位置初始化
	int width = 25, height = 25;
	std::vector<Point2f> vertices_origin = { Point2f(0,0),Point2f(0,width),Point2f(height, width), Point2f(height, 0) };
	std::vector<Point2f> vertices_current(4);
	for (int i = 0; i < 4; i++) {
		vertices_current[i] = Point2f(contour[i].x, contour[i].y);
	}
	int corners_count = (width - 1) * (height - 1);
	Point3d *corners_origin = new Point3d[corners_count];
	int counter = 0;
	for (int i = 1; i < width; i++) {
		for (int j = 1; j < height; j++) {
			corners_origin[counter] = Point3d(i, j, 1);
			counter++;
		}
	}
	Mat coordinates(corners_count, 3, CV_64FC1, corners_origin);
	//  2.2 计算透视投影矩阵
	Mat T = getPerspectiveTransform(vertices_origin, vertices_current);
	//  2.3 计算变换后的角点
	Mat corners = T * coordinates.t();
	//  2.4 计入output
	output.resize(corners_count);
	double* ptr = corners.ptr<double>();
	for (int i = 0; i < corners_count; i++) {
		double w = *(ptr + 2 * corners.cols);
		output[i] = Point2f((*ptr) / w, (*(ptr + corners.cols)) / w);
		ptr++;
	}
	delete[] corners_origin;
}

// 方案3：
// step1: 做纵横方向的投影
// step2: 找峰值确定中心位置
void bydirectionProj(cv::Mat& input, std::vector<cv::Mat> output) {
	CV_Assert(input.size() == output[0].size() && input.size() == output[1].size());
	int rows = input.rows;
	int cols = input.cols;
	std::vector<int> y_proj(rows, 0);
	std::vector<int> x_proj(cols, 0);
	for (int i = 0; i < rows; i++) {
		uchar* ptr = input.ptr(i);
		for (int j = 0; j < cols; j++) {
			if (*ptr == 255) {
				y_proj[i]++;
				x_proj[j]++;
			}
			ptr++;
		}
	}
	int x_total = 0, y_total = 0;
	for (int k = 0; k < x_proj.size(); k++) {
		x_total += x_proj[k];
		line(output[0], Point(k, 0), Point(k, x_proj[k]), 255);
	}
	for (int k = 0; k < y_proj.size(); k++) {
		y_total += y_proj[k];
		line(output[1], Point(0, k), Point(y_proj[k], k), 255);
	}
}

// 方案2改进：
// step1: 透视投影变换获取初始角点位置
// step2: 使用模板匹配找到角点精确位置
void detectCorner(cv::Mat& canva, cv::Mat bw, cv::Size size, std::vector<cv::Point2f>& output, double ws, bool TM_ON) {
	// 1.找出近似四边形
	std::vector<std::vector<Point>> contours;
	findContours(bw, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	std::vector<Point> contour = contours[0];
	bool closed = true;
	approxPolyDP(contour, contour, 40, closed);
	for (int i = 0; i < contour.size(); i++) {
		line(canva, contour[i], contour[(i + 1) % contour.size()], Scalar(0, 0, 255));
	}
	// 2.计算交点
	//  2.1 原始坐标位置初始化
	int width = size.width, height = size.height;
	std::vector<Point2f> vertices_origin = { Point2f(0,0),Point2f(0,width),Point2f(height, width), Point2f(height, 0) };
	std::vector<Point2f> vertices_current(4);
	for (int i = 0; i < 4; i++) {
		vertices_current[i] = Point2f(contour[i].x, contour[i].y);
	}
	//  2.2 计算透视投影矩阵
	Mat T = getPerspectiveTransform(vertices_origin, vertices_current);

	//  2.3 逐个计算初始角点位置与精确角点位置
	std::vector<Point3d> actual_points;
	for (int i = 1; i < width; i++) {
		for (int j = 1; j < height; j++) {
			actual_points.emplace_back(i, j, 1);
		}
	}
	//Point2f corner_point = corner_finder(bw, actual_points[5], T, ws, TM_ON, 5);
	//output.push_back(corner_point);

	int count = 0;
	for (Point3d current_point : actual_points)
	{
		Point2f corner_point = corner_finder(bw, current_point, T, ws, TM_ON, count);
		count++;
		output.push_back(corner_point);
	}
}

void detectCorner(cv::Mat bw, cv::Size size, std::vector<cv::Point2f>& output, double ws, bool TM_ON) {
	// 1.找出近似四边形
	std::vector<std::vector<Point>> contours;
	findContours(bw, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	std::vector<Point> contour = contours[0];
	bool closed = true;
	approxPolyDP(contour, contour, 40, closed);

	// 2.计算交点
	//  2.1 原始坐标位置初始化
	int width = size.width, height = size.height;
	std::vector<Point2f> vertices_origin = { Point2f(0,0),Point2f(0,width),Point2f(height, width), Point2f(height, 0) };
	std::vector<Point2f> vertices_current(4);
	for (int i = 0; i < 4; i++) {
		vertices_current[i] = Point2f(contour[i].x, contour[i].y);
	}
	//  2.2 计算透视投影矩阵
	Mat T = getPerspectiveTransform(vertices_origin, vertices_current);

	//  2.3 逐个计算初始角点位置与精确角点位置
	std::vector<Point3d> actual_points;
	for (int i = 1; i < width; i++) {
		for (int j = 1; j < height; j++) {
			actual_points.emplace_back(i, j, 1);
		}
	}
	//Point2f corner_point = corner_finder(bw, actual_points[5], T, ws, TM_ON, 5);
	//output.push_back(corner_point);

	int count = 0;
	for (Point3d current_point : actual_points)
	{
		Point2f corner_point = corner_finder(bw, current_point, T, ws, TM_ON, count);
		count++;
		output.push_back(corner_point);
	}
}

void detectCorner_fitting(cv::Mat& canva, cv::Mat bw, cv::Size size, std::vector<cv::Point2f>& output, double ws) {
	// 1.找出近似四边形
	std::vector<std::vector<Point>> contours;
	findContours(bw, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	std::vector<Point> contour = contours[0];
	bool closed = true;
	approxPolyDP(contour, contour, 120, closed);
	int minx = INT_MAX, miny = INT_MAX, maxx = 0, maxy = 0;
	for (int i = 0; i < contour.size(); i++) {
		line(canva, contour[i], contour[(i + 1) % contour.size()], Scalar(0, 255, 0), 3);
		if (contour[i].x < minx)
			minx = contour[i].x;
		if (contour[i].x > maxx)
			maxx = contour[i].x;
		if (contour[i].y < miny)
			miny = contour[i].y;
		if (contour[i].y > maxy)
			maxy = contour[i].y;
	}
	namedWindow("approx quad", WINDOW_NORMAL);
	imshow("approx quad", canva);
	int key = waitKey();
	if (key == 's') {
		imwrite("E:/PictureCollection/处理结果/论文1/approxQuad.png", canva(Rect(Point(minx-10, miny-10), Point(maxx+10, maxy+10))));
	}

	Point2d direction1 = contour[1] - contour[0];
	Point2d direction2 = contour[2] - contour[1];
	direction1 = direction1 / norm(direction1);
	direction2 = direction2 / norm(direction2);
	double theta1 = acos(direction1.x);
	double theta2 = acos(direction2.x);
	Vec2d theta(theta1, theta2);
	// 2.计算交点
	//  2.1 原始坐标位置初始化
	int width = size.width, height = size.height;
	std::vector<Point2f> vertices_origin = { Point2f(0,0),Point2f(0,width),Point2f(height, width), Point2f(height, 0) };
	std::vector<Point2f> vertices_current(4);
	for (int i = 0; i < 4; i++) {
		vertices_current[i] = Point2f(contour[i].x, contour[i].y);
	}
	//  2.2 计算透视投影矩阵
	Mat T = getPerspectiveTransform(vertices_origin, vertices_current);

	//  2.3 逐个计算初始角点位置与精确角点位置
	std::vector<Point3d> actual_points;
	for (int i = 1; i < height; i++) {
		for (int j = 1; j < width; j++) {
			actual_points.emplace_back(j, i, 1);
		}
	}
	//Point2f corner_point = corner_finder_fitting(canva, bw, actual_points[5], T, ws, -1);
	//output.push_back(corner_point);

	int count = 0;
	for (;count<actual_points.size();count++)
	{
		Point2f corner_point = corner_finder_fitting(canva, bw, actual_points[count], T, ws, count);
#ifdef NDEBUG
		std::cout << "..." << count+1 << "/" << actual_points.size() << "\r";
#endif
		output.push_back(corner_point);
	}
}

void detectCorner_fitting(cv::Mat bw, cv::Size size, std::vector<cv::Point2f>& output, double ws) {
	// 1.找出近似四边形
	std::vector<std::vector<Point>> contours;
	findContours(bw, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	std::vector<Point> contour = contours[0];
	bool closed = true;
	approxPolyDP(contour, contour, 120, closed);

	// 2.计算交点
	//  2.1 原始坐标位置初始化
	int width = size.width, height = size.height;
	std::vector<Point2f> vertices_origin = { Point2f(0,0),Point2f(0,width),Point2f(height, width), Point2f(height, 0) };
	std::vector<Point2f> vertices_current(4);
	for (int i = 0; i < 4; i++) {
		vertices_current[i] = Point2f(contour[i].x, contour[i].y);
	}
	//  2.2 计算透视投影矩阵
	Mat T = getPerspectiveTransform(vertices_origin, vertices_current);

	//  2.3 逐个计算初始角点位置与精确角点位置
	std::vector<Point3d> actual_points;
	for (int i = 1; i < height; i++) {
		for (int j = 1; j < width; j++) {
			actual_points.emplace_back(j, i, 1);
		}
	}
	//Point2f corner_point = corner_finder(bw, actual_points[5], T, ws, TM_ON, 5);
	//output.push_back(corner_point);

	model_fitting(bw, T, ws, actual_points, output);
//	int count = 0;
//	for (Point3d current_point : actual_points)
//	{
//		Point2f corner_point = corner_finder_fitting(bw, current_point, T, ws, count);
//		count++;
//#ifdef NDEBUG
//		std::cout << "..." << count << "/" << actual_points.size() << "\r";
//#endif
//		output.push_back(corner_point);
//	}
}

void detectCornerPair_fitting(cv::Mat bw1, cv::Mat bw2, cv::Size size,
	std::vector<cv::Point2f>& corners1, std::vector<cv::Point2f>& corners2, double ws) {
	// 1.找出近似四边形
	std::vector<std::vector<Point>> contours1, contours2;
	findContours(bw1, contours1, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	findContours(bw2, contours2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	approxPolyDP(contours1[0], contours1[0], 40, true);
	approxPolyDP(contours2[0], contours2[0], 40, true);
	Point2d direction1 = contours1[0][1] - contours1[0][0];
	Point2d direction2 = contours2[0][1] - contours2[0][0];
	int shift1 = 0, shift2 = 0;
	double crossP = direction1.cross(direction2);
	double sinRadDiff = crossP / (norm(direction1) * norm(direction2));
	if (abs(sinRadDiff) > sin(10 * CV_PI / 180)) {
		if (sinRadDiff > 0)
			shift1 = 1;
		else
			shift2 = 1;
	}
	int width = size.width, height = size.height;
	std::vector<Point2f> vertices_origin = { Point2f(0,0),Point2f(0,width),Point2f(height, width), Point2f(height, 0) };
	std::vector<Point2f> vertices1(4), vertices2(4);
	for (int i = 0; i < 4; i++) {
		vertices1[i] = Point2f(contours1[0][(i+shift1)%4].x, contours1[0][(i+shift1)%4].y);
		vertices2[i] = Point2f(contours2[0][(i + shift2) % 4].x, contours2[0][(i+shift2)%4].y);
	}
	Mat T1 = getPerspectiveTransform(vertices_origin, vertices1);
	Mat T2 = getPerspectiveTransform(vertices_origin, vertices2);

	std::vector<Point3d> actual_points;
	for (int i = 1; i < height; i++) {
		for (int j = 1; j < width; j++) {
			actual_points.emplace_back(j, i, 1);
		}
	}

	model_fitting(bw1, T1, ws, actual_points, corners1);
	model_fitting(bw2, T2, ws, actual_points, corners2);
}

// method2: segment image base on adaptive thresholding 
Mat ATH_impl(cv::Mat& input_gray, int ws, int thresh) {
	CV_Assert(input_gray.type() == CV_8U);
	Mat output_bw(input_gray.size(), CV_8U);
	// generate integral image
	Mat base = Mat::zeros(input_gray.size() + Size(1, 1), CV_32F);
	Mat intImg(base, Rect(1,1,input_gray.cols, input_gray.rows));
	float* prev_row_ptr = nullptr;
	for (int i = 0; i < intImg.rows; i++) {
		float* curr_row_ptr = intImg.ptr<float>(i);
		uchar* corr_row_ptr = input_gray.ptr(i);
		float accumulation = 0;
		for (int j = 0; j < intImg.cols; j++) {
			accumulation += corr_row_ptr[j];
			curr_row_ptr[j] = accumulation + (prev_row_ptr ? prev_row_ptr[j] : 0);			
		}
		prev_row_ptr = curr_row_ptr;
	}
	Mat conv = Mat::zeros(intImg.size(), CV_32F);
	float maxv=0;
	for (int i = 0; i < intImg.rows; i++) {
		float* conv_res = conv.ptr<float>(i);
		uchar* intensity = input_gray.ptr(i);
		uchar* res = output_bw.ptr(i);
		for (int j = 0; j < intImg.cols; j++) {
			// calculate vertex
			int r1 = i - ws / 2;
			int r2 = i + ws / 2 + 1;
			int c1 = j - ws / 2;
			int c2 = j + ws / 2 + 1;
			// border checking
			r1 = r1 < 0 ? 0 : r1;
			c1 = c1 < 0 ? 0 : c1;
			r2 = r2 > intImg.rows ? intImg.rows : r2;
			c2 = c2 > intImg.cols ? intImg.cols : c2;
			// thresholding on each pixel
			int count = (r2 - r1) * (c2 - c1);
			float acc_in_window = base.at<float>(r2, c2) - base.at<float>(r1, c2) - base.at<float>(r2, c1) + base.at<float>(r1, c1);
			float avg_I = acc_in_window / count;
			res[j] = abs(intensity[j]-avg_I)>thresh ? 255 : 0;
			conv_res[j] = abs(intensity[j] - avg_I);
			if (conv_res[j] > maxv)
				maxv = conv_res[j];
		}
	}
#ifdef _DEBUG
	conv.convertTo(conv, CV_8U, 255 / maxv);
	imshow("conv", conv);
	int key = waitKey();
	if (key == 's') {
		std::string filename;
		std::cout << "Please input the file name:\n";
		std::cin >> filename;
		imwrite(filename, conv);
	}
#endif
	return output_bw;
}

static int limitToRange(int v, int min, int max) {
	if (v < min)
		return min;
	else if (v > max)
		return max;
	else
		return v;
}
static int findMedian(Mat& gray) {
	Mat hist;
	int channels = 0;
	int histsize = 256;
	float range[] = { 0,256 };
	const float* ranges[] = {range};
	calcHist(&gray, 1, &channels, Mat(), hist, 1, &histsize, ranges);
	int half = gray.total() / 2;
	float counter = 0;
	float* ptr = hist.ptr<float>();
	for (int i = 0; i < hist.total(); i++) {
		counter += ptr[i];
		if (counter > half)
			return i;
	}
	return -1;
}
Mat ATH_medianBased(Mat& gray, int ws) {
	//int minx = ws / 2,
	//	maxx = gray.cols - 1 - ws / 2,
	//	miny = ws / 2,
	//	maxy = gray.rows - 1 - ws / 2;
	//Mat bw = Mat::zeros(gray.size(), CV_8U);
	//for (int i = 0; i < bw.rows; i++) {
	//	uchar* ptr = bw.ptr(i);
	//	uchar* data_p = gray.ptr(i);
	//	for (int j = 0; j < bw.cols; j++) {
	//		Point center(limitToRange(j, minx, maxx), limitToRange(i, miny, maxy));
	//		Mat wnd(gray, Rect(center - Point(minx, miny), center + Point(minx, miny)));
	//		int median = findMedian(wnd);
	//		int thresh = 10;
	//		if (abs(data_p[j] - median) > thresh) {
	//			ptr[j] = 255;
	//		}
	//	}
	//}
	Mat median, bw;
	medianBlur(gray, median, ws);
	absdiff(gray, median, bw);
	threshold(bw, bw, 10, 255, THRESH_BINARY);
#ifdef _DEBUG
	imshow("medianBlur", median);
	waitKey();
#endif
	return bw;
}

// use for eliminate wrong wave after fourier transform
static bool is_eliminated(int label, std::vector<int> list) {
	for (int elem : list) {
		if (label == elem)
			return true;
	}
	return false;
}

static std::vector<int> getLUT(int step, int ws) {
	std::vector<int> LUT;
	for (int i = -ws/2; i <= ws/2; i++) {
		for (int j = -ws/2; j <= ws/2; j++) {
			LUT.push_back(i * step + j);
		}
	}
	return LUT;
}

static std::vector<int> eliminateBigError(Mat& label) {
	std::vector<int> eliminated = { 0 };
	int step = label.cols;
	int ws = 11;
	std::vector<int> lut = getLUT(step, ws);
	uchar counter = 0;
	for (int i = ws/2; i < label.rows-ws/2; i++) {
		int* label_ptr = label.ptr<int>(i);
		for (int j = ws/2; j < label.cols-ws/2; j++) {
			int _label = label_ptr[j];
			if (is_eliminated(_label, eliminated))
				continue;
			else {
				uchar same_count = 0;
				for (int k:lut) {
					if (*(label_ptr + j + k) == _label)
						same_count++;
				}
				if (same_count >= 121) {
					//printf("count:%d\nlabel:%d\n", same_count, _label);
					eliminated.push_back(_label);
				}
			}
		}
	}
	return eliminated;
}

static void showEliminated(Mat& label, std::vector<int> eliminated) {
	Mat bw = Mat::zeros(label.size(), CV_8U);
	int* _label = (int*)label.data;
	uchar* _bw = bw.data;
	for (int i = 0; i < label.total(); i++) {
		if (!is_eliminated(*_label, eliminated))
			*_bw = 255;
		_label++;
		_bw++;
	}
	namedWindow("eliminated", WINDOW_NORMAL);
	imshow("eliminated", bw);
	int key = waitKey();
	if (key == 's') {
		std::string filename;
		std::cout << "Please input the file name:\n";
		std::cin >> filename;
		imwrite(filename, bw);
	}
}
// ende

Mat segment_ref(Mat& gray) {
	Mat* complexI = fourierTransform(gray);	//返回一个记录实部虚部的二通道的矩阵
	Mat* filter = hpFilter(gray.size(), 30);	//生成高通滤波器
	fftshift(*complexI);	//将坐标原点移至中心
	mulSpectrums(*complexI, *filter, *complexI, 0);
	fftshift(*complexI);
	idft(*complexI, *complexI);
	Mat planes[2];
	split(*complexI, planes);
	normalize(planes[0], planes[0], 0, 1, NORM_MINMAX);

	Mat image = planes[0] * 255;
	image.convertTo(image, CV_8U);
	equalizeHist(image, image);
	//namedWindow("eq", WINDOW_NORMAL);
	//imshow("eq", image);


	Mat ref = ATH_impl(gray, 30, 15);
	Mat bw = ATH_impl(image, 54, 124);
	//namedWindow("ref", WINDOW_NORMAL);
	//namedWindow("bw", WINDOW_NORMAL);
	//imshow("ref", ref);
	//imshow("bw", bw);
	Mat grid;
	extractGrid(bw, ref, grid);	
	delete complexI;
	delete filter;
	return grid;
}

Mat segment(Mat& gray) {
	Mat* complexI = fourierTransform(gray);	//返回一个记录实部虚部的二通道的矩阵
	Mat* filter = hpFilter(gray.size(), 30);	//生成高通滤波器
	fftshift(*complexI);	//将坐标原点移至中心
	mulSpectrums(*complexI, *filter, *complexI, 0);
	fftshift(*complexI);
	idft(*complexI, *complexI);
	Mat planes[2];
	split(*complexI, planes);
	normalize(planes[0], planes[0], 0, 1, NORM_MINMAX);

	Mat image = planes[0] * 255;
	image.convertTo(image, CV_8U);
	equalizeHist(image, image);
	//namedWindow("eq", WINDOW_NORMAL);
	//imshow("eq", image);


	Mat bw = ATH_impl(image, 54, 110);
	//namedWindow("ref", WINDOW_NORMAL);
	//namedWindow("bw", WINDOW_NORMAL);
	//imshow("ref", ref);
	//imshow("bw", bw);
	Mat grid;
	extractGrid(bw, grid);
	delete complexI;
	delete filter;
	return grid;
}

void extractGrid(cv::Mat& input_bw, cv::Mat& output, std::vector<Point>& out_points) {
	Mat labels = Mat::zeros(input_bw.size(), CV_32S);
	int number = connectedComponents(input_bw, labels, 8, CV_32S, CCL_BBDT);	//BBDT is the fastest method in test
	int* areas = new int[number];
	memset(areas, 0, sizeof(int) * number);
	int* ptr = labels.ptr<int>(0);
	for (int i = 0; i < input_bw.total(); i++) {
		if (ptr[i] == 0) {
			continue;
		}
		areas[ptr[i]] += 1;
	}
	int max = areas[1];
	int idx = 1;
	for (int i = 0; i < number; i++) {
		if (areas[i] > max) {
			max = areas[i];
			idx = i;
		}
	}
	Mat bw = Mat::zeros(input_bw.size(), CV_8U);
	for (int i = 0; i < bw.total(); i++) {
		if (ptr[i] == idx)
		{
			bw.data[i] = 255;
			out_points.emplace_back(i % bw.cols, int(i / bw.cols));
		}
	}
	delete[] areas;
	output = bw;
}

void extractGrid(cv::Mat& input_bw, cv::Mat& ref, Mat& output) {
	Mat labels = Mat::zeros(input_bw.size(), CV_32S);
	int number = connectedComponents(input_bw, labels, 8, CV_32S, CCL_BBDT);	//BBDT is the fastest method in test
	std::vector<int> ignored = eliminateBigError(labels);
	//showEliminated(labels, ignored);
	int* areas = new int[number];
	memset(areas, 0, sizeof(int) * number);
	int* ptr = labels.ptr<int>(0);
	for (int i = 0; i < input_bw.total(); i++) {
		if (is_eliminated(ptr[i], ignored)) {
			continue;
		}
		areas[ptr[i]] += 1;
	}
	std::pair<int, int> top3[3];
	memset(top3, 0, sizeof(std::pair<int, int>) * 3);
	for (int i = 0; i < number; i++) {
		if (areas[i] > top3[0].second) {
			top3[2] = top3[1];
			top3[1] = top3[0];
			top3[0] = std::pair<int, int>(i, areas[i]);
		}
		else if (areas[i] > top3[1].second) {
			top3[2] = top3[1];
			top3[1] = std::pair<int, int>(i, areas[i]);
		}
		else if (areas[i] > top3[2].second) {
			top3[2] = std::pair<int, int>(i, areas[i]);
		}
	}
	delete[] areas;
	uchar* p_ref = ref.data;
	for (int k = 0; k < 3; k++)
	{
		int acc = 0;
		Mat bw = Mat::zeros(input_bw.size(), CV_8U);
		for (int i = 0; i < bw.total(); i++) {
			if (ptr[i] == top3[k].first)
			{
				bw.data[i] = 255;
				acc = acc + p_ref[i] / 255;
			}
		}
		if (acc > 0.5 * top3[k].second) {
			output = bw;
			return;
		}
	}
	output = Mat::zeros(input_bw.size(), CV_8U);
	return;
}

void extractGrid(cv::Mat& input_bw, cv::Mat& output) {
	Mat labels = Mat::zeros(input_bw.size(), CV_32S);
	int number = connectedComponents(input_bw, labels, 8, CV_32S, CCL_BBDT);	//BBDT is the fastest method in test
	std::vector<int> ignored = eliminateBigError(labels);
	int* areas = new int[number];
	memset(areas, 0, sizeof(int) * number);
	int* ptr = labels.ptr<int>(0);
	for (int i = 0; i < input_bw.total(); i++) {
		if (is_eliminated(ptr[i], ignored)) {
			continue;
		}
		areas[ptr[i]] += 1;
	}
	int max = areas[1];
	int idx = 1;
	for (int i = 0; i < number; i++) {
		if (areas[i] > max) {
			max = areas[i];
			idx = i;
		}
	}
	Mat bw = Mat::zeros(input_bw.size(), CV_8U);
	for (int i = 0; i < bw.total(); i++) {
		if (ptr[i] == idx)
		{
			bw.data[i] = 255;
		}
	}
	delete[] areas;
	output = bw;
}