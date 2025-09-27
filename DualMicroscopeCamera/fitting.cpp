#include "fitting.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

//功能：计算图像的积分值（所有像素值的归一化累加和）
//输入：灰度图像 img
//输出：积分值（范围在[0, 图像像素总数]）
//意义：用于评估图像的整体亮度或能量，可能在后续步骤中用于归一化或优化目标函数
static double imgIntegration(Mat& img) {
    double res = 0; 
    uchar* data = img.data; 
    for (size_t i = 0; i < img.total(); i++) {
        // 将每个像素值转换为[0, 1]范围内的浮点数并累加到结果中
        res = res + double(data[i]) / 255;
    }
    return res;
}

//功能：根据给定的坐标 coord 和参数 params，计算模型的预测值
//模型形式：
//参数：params = [a1, a2, cx, cy, b1, b2]，其中：a1, a2：两个方向的角度;cx, cy：中心点坐标;b1, b2：线宽参数
//公式：通过逻辑函数（Sigmoid）组合两个交叉方向的直线，模拟交叉线的强度分布
//输出：模型在坐标 coord 处的预测值（范围[0, 1]）
//意义：定义了一个双线性交叉的数学模型，用于拟合图像中的交叉结构（如棋盘格角点）
static double calcModelValue(Point2d coord, const Vec6d& params) {
    // 从参数向量中提取各个参数
    const double &a1 = params[0], &a2 = params[1], &cx = params[2], &cy = params[3], &b1 = params[4], &b2 = params[5];
    // 计算两个方向的系数
    Point2d coeff1(cos(a1), sin(a1));
    Point2d coeff2(cos(a2), sin(a2));
    // 计算两个方向的距离
    double l1 = coeff1.cross(coord) - coeff1.cross(Point2d(cx, cy));
    double l2 = coeff2.cross(coord) - coeff2.cross(Point2d(cx, cy));
    // 计算四个逻辑函数的值
    double f1 = 1 / (1 + exp(-10 * (l1 + b1 / 2)));
    double f2 = 1 / (1 + exp(10 * (l1 - b1 / 2)));
    double f3 = 1 / (1 + exp(-10 * (l2 + b2 / 2)));
    double f4 = 1 / (1 + exp(10 * (l2 - b2 / 2)));
    // 返回模型值
    return f1 * f2 + f3 * f4 - f1 * f2 * f3 * f4;
}

//功能：根据参数生成模板图像
//输入：size：模板尺寸;params：模型参数;ws：窗口大小（用于调整中心坐标）
//输出：生成的模板图像（CV_8U 类型）
//意义：用于模板匹配（matchTemplate），在粗略拟合阶段快速定位交叉结构的大致位置
static Mat getTempl(Size size, const Vec6d& params, int ws) {
    int shift = ws / 2; // 计算模板的偏移量
    Mat dst(size, CV_8U); // 创建目标模板图像
    // 从参数向量中提取各个参数
    const double& a1 = params[0], & a2 = params[1], & b1 = params[4], & b2 = params[5];
    double cx = params[2], cy = params[3];
    // 调整中心坐标
    cx -= shift;
    cy -= shift;
    // 计算两个方向的系数
    Point2d coeff1(cos(a1), sin(a1));
    Point2d coeff2(cos(a2), sin(a2));
    // 遍历模板图像的每个像素
    for (int i = 0; i < dst.rows; i++) {
        uchar* ptr = dst.ptr(i); // 获取当前行的指针
        for (int j = 0; j < dst.cols; j++) {
            Point2d coord(j, i); // 当前像素的坐标
            // 计算两个方向的距离
            double l1 = coeff1.cross(coord) - coeff1.cross(Point2d(cx, cy));
            double l2 = coeff2.cross(coord) - coeff2.cross(Point2d(cx, cy));
            // 计算四个逻辑函数的值
            double f1 = 1 / (1 + exp(-10 * (l1 + b1 / 2)));
            double f2 = 1 / (1 + exp(10 * (l1 - b1 / 2)));
            double f3 = 1 / (1 + exp(-10 * (l2 + b2 / 2)));
            double f4 = 1 / (1 + exp(10 * (l2 - b2 / 2)));
            // 计算当前像素的值并赋值
            ptr[j] = 255 * (f1 * f2 + f3 * f4 - f1 * f2 * f3 * f4);
        }
    }
    return dst; // 返回生成的模板图像
}

// 计算图像与模型之间的相关性得分
static double correlationScore(Mat& img, Vec6d params) {
    double dst = 0; // 初始化相关性得分
    // 遍历图像的每一行
    for (int i = 0; i < img.rows; i++) {
        uchar* ptr = img.ptr(i); // 获取当前行的指针
        // 遍历当前行的每一列
        for (int j = 0; j < img.cols; j++) {
            double model = calcModelValue(Point2d(j, i), params); // 计算模型值
            double data = (double)ptr[j]; // 获取图像数据
            dst += model * data; // 累加模型值与图像数据的乘积
        }
    }
    return dst; // 返回相关性得分
}

// 在图像上绘制直线（默认颜色为白色，线宽为1）
static void drawLine(Mat& img, double a, double b, double c) {
    Point2d p1, p2; // 定义两个点
    p1.x = 0; // 第一个点的x坐标
    p2.x = img.cols; // 第二个点的x坐标
    p1.y = -c / b; // 第一个点的y坐标
    p2.y = -(a * p2.x + c) / b; // 第二个点的y坐标
    line(img, p1, p2, 255); // 绘制直线，颜色为白色
}

// 在图像上绘制直线（指定颜色和线宽）
static void drawLine(Mat& img, Scalar color, double a, double b, double c, double thickness) {
    Point2d p1, p2; // 定义两个点
    p1.x = 0; // 第一个点的x坐标
    p2.x = img.cols; // 第二个点的x坐标
    p1.y = -c / b; // 第一个点的y坐标
    p2.y = -(a * p2.x + c) / b; // 第二个点的y坐标
    line(img, p1, p2, color, thickness); // 绘制直线，指定颜色和线宽
}

// 粗略拟合图像中的模型
void coarseFitting(cv::Mat& img, cv::Vec6d& params) {
    int ws = 11; // 模板窗口大小
    Mat scoreMap; // 分数图
    // 获取模板图像
    Mat templ = getTempl(img.size() - Size(ws-1, ws-1), params, ws);
    Point max_pos; // 最大匹配位置
    Point center(ws / 2, ws / 2); // 模板中心
    // 匹配模板
    matchTemplate(img, templ, scoreMap, TM_CCORR_NORMED);
    // 找到分数图中的最大值位置
    minMaxLoc(scoreMap, nullptr, 0, nullptr, &max_pos);
        
    int dx = (max_pos - Point(ws / 2, ws / 2)).x; // 计算x方向偏移
    int dy = (max_pos - Point(ws / 2, ws / 2)).y; // 计算y方向偏移
    params[2] += dx; // 更新参数x坐标
    params[3] += dy; // 更新参数y坐标
}

// 粗略拟合图像中的模型（带可视化）
void coarseFittingV(cv::Mat& img, cv::Vec6d& params) {
    int ws = 11; // 模板窗口大小windowsize
    Mat scoreMap; // 分数图
    // 获取模板图像
    Mat templ = getTempl(img.size() - Size(ws - 1, ws - 1), params, ws);
    Mat canva2; // 可视化模板
    resize(templ, canva2, Size(), 30, 30, INTER_NEAREST); // 缩放模板
    imshow("templ", canva2); // 显示模板
    int key = waitKey(); // 等待按键
    if (key == 's') { // 如果按键为's'
        printf("Please enter the path to save image:\n"); // 提示输入保存路径
        std::string p; // 保存路径
        std::cin >> p; // 输入保存路径
        cv::imwrite(p, canva2); // 保存模板图像
    }
    Point max_pos; // 最大匹配位置
    Point center(ws / 2, ws / 2); // 模板中心
    // 匹配模板
    matchTemplate(img, templ, scoreMap, TM_CCORR_NORMED);
    Mat canva; // 可视化分数图
    scoreMap.convertTo(canva, CV_8U, 255); // 转换分数图为8位图像
    resize(canva, canva, Size(500, 500), 0, 0, INTER_NEAREST); // 缩放分数图
    imshow("score", canva); // 显示分数图
    key = waitKey(); // 等待按键
    if (key == 's') { // 如果按键为's'
        printf("Please enter the path to save image:\n"); // 提示输入保存路径
        std::string p; // 保存路径
        std::cin >> p; // 输入保存路径
        cv::imwrite(p, canva); // 保存分数图
    }
    // 找到分数图中的最大值位置
    minMaxLoc(scoreMap, nullptr, 0, nullptr, &max_pos);

    int dx = (max_pos - Point(ws / 2, ws / 2)).x; // 计算x方向偏移
    int dy = (max_pos - Point(ws / 2, ws / 2)).y; // 计算y方向偏移
    params[2] += dx; // 更新参数x坐标
    params[3] += dy; // 更新参数y坐标
}

// 计算图像梯度函数，参数为4维向量
Vec4d getGrad(Mat& img, Vec4d params, double* err_ptr = nullptr) {
    // 提取参数
    double a1 = params[0], a2 = params[1], cx = params[2], cy = params[3];
    // 初始化梯度向量
    Vec4d grad(0, 0, 0, 0);
    // 初始化误差
    double err = 0;
    // 遍历图像的每一行
    for (int i = 0; i < img.rows; i++) {
        // 获取当前行的数据指针
        uchar* data = img.ptr<uchar>(i);
        // 遍历当前行的每一列
        for(int j=0;j<img.cols;j++) {
            // 当前像素的坐标
            Point2d coord(j, i);
            // 计算方向向量
            Point2d coeff1(cos(a1), sin(a1));
            Point2d coeff2(cos(a2), sin(a2));
            // 计算距离
            double l1 = coeff1.cross(coord) - coeff1.cross(Point2d(cx, cy));
            double l2 = coeff2.cross(coord) - coeff2.cross(Point2d(cx, cy));
            // 计算偏导数
            double pl1pa1 = Point2d(-sin(a1), cos(a1)).cross(coord) - Point2d(-sin(a1), cos(a1)).cross(Point2d(cx, cy));
            double pl2pa2 = Point2d(-sin(a2), cos(a2)).cross(coord) - Point2d(-sin(a2), cos(a2)).cross(Point2d(cx, cy));
            // 计算软阈值函数及其导数
            double g1 = exp(-10 * (l1 + 1.5));
            double g2 = exp(10 * (l1 - 1.5));
            double g3 = exp(-10 * (l2 + 1.5));
            double g4 = exp(10 * (l2 - 1.5));
            double f1 = 1 / (1 + g1);
            double f2 = 1 / (1 + g2);
            double f3 = 1 / (1 + g3);
            double f4 = 1 / (1 + g4);
            double pf1pl1 = 10 * g1 / ((1 + g1) * (1 + g1));
            double pf2pl1 = -10 * g2 / ((1 + g2) * (1 + g2));
            double pf3pl2 = 10 * g3 / ((1 + g3) * (1 + g3));
            double pf4pl2 = -10 * g4 / ((1 + g4) * (1 + g4));
            // 计算复合函数的导数
            double pfpl1 = (1 - f3 * f4) * (f2 * pf1pl1 + f1 * pf2pl1);
            double pfpl2 = (1 - f1 * f2) * (f4 * pf3pl2 + f3 * pf4pl2);
            // 计算模型函数值
            double f = f1 * f2 + f3 * f4 - f1 * f2 * f3 * f4;
            // 计算误差
            double D = data[j] / 255 - f;
            // 计算梯度
            double pLpa1 = -2 * D * pfpl1 * pl1pa1;
            double pLpa2 = -2 * D * pfpl2 * pl2pa2;
            double pLpcx = -2 * D * (pfpl1 * sin(a1) + pfpl2 * sin(a2));
            double pLpcy = -2 * D * (pfpl1 * (-cos(a1)) + pfpl2 * (-cos(a2)));
            // 累加梯度
            grad += Vec4d(pLpa1, pLpa2, pLpcx, pLpcy);
            // 累加误差
            err = err + D * D;
        }
    }
    // 如果提供了误差指针，则更新误差
    if (err_ptr)
        *err_ptr = err;
    // 返回梯度
    return grad;
}

// 计算图像梯度函数，参数为6维向量
Vec6d getGrad(Mat& img, Vec6d params, double* err_ptr = nullptr) {
    // 提取参数
    double a1 = params[0], a2 = params[1], cx = params[2], cy = params[3], b1 = params[4], b2 = params[5];
    // 初始化梯度向量
    Vec6d grad(0, 0, 0, 0, 0, 0);
    // 初始化误差
    double err = 0;
    // 遍历图像的每一行
    for (int i = 0; i < img.rows; i++) {
        // 获取当前行的数据指针
        uchar* data = img.ptr<uchar>(i);
        // 遍历当前行的每一列
        for (int j = 0; j < img.cols; j++) {
            // 当前像素的坐标
            Point2d coord(j, i);
            // 计算方向向量
            Point2d coeff1(cos(a1), sin(a1));
            Point2d coeff2(cos(a2), sin(a2));
            // 计算距离
            double l1 = coeff1.cross(coord) - coeff1.cross(Point2d(cx, cy));
            double l2 = coeff2.cross(coord) - coeff2.cross(Point2d(cx, cy));
            // 计算偏导数
            double pl1pa1 = Point2d(-sin(a1), cos(a1)).cross(coord) - Point2d(-sin(a1), cos(a1)).cross(Point2d(cx, cy));
            double pl2pa2 = Point2d(-sin(a2), cos(a2)).cross(coord) - Point2d(-sin(a2), cos(a2)).cross(Point2d(cx, cy));
            // 计算软阈值函数及其导数
            double g1 = exp(-10 * (l1 + b1 / 2));
            double g2 = exp(10 * (l1 - b1 / 2));
            double g3 = exp(-10 * (l2 + b2 / 2));
            double g4 = exp(10 * (l2 - b2 / 2));
            double f1 = 1 / (1 + g1);
            double f2 = 1 / (1 + g2);
            double f3 = 1 / (1 + g3);
            double f4 = 1 / (1 + g4);
            double pf1pl1 = 10 * g1 / ((1 + g1) * (1 + g1));
            double pf2pl1 = -10 * g2 / ((1 + g2) * (1 + g2));
            double pf3pl2 = 10 * g3 / ((1 + g3) * (1 + g3));
            double pf4pl2 = -10 * g4 / ((1 + g4) * (1 + g4));
            // 计算复合函数的导数
            double pfpl1 = (1 - f3 * f4) * (f2 * pf1pl1 + f1 * pf2pl1);
            double pfpl2 = (1 - f1 * f2) * (f4 * pf3pl2 + f3 * pf4pl2);
            // 计算模型函数值
            double f = f1 * f2 + f3 * f4 - f1 * f2 * f3 * f4;
            // 计算误差
            double D = data[j] / 255 - f;
            // 计算梯度
            double pLpa1 = -2 * D * pfpl1 * pl1pa1;
            double pLpa2 = -2 * D * pfpl2 * pl2pa2;
            double pLpcx = -2 * D * (pfpl1 * sin(a1) + pfpl2 * sin(a2));
            double pLpcy = -2 * D * (pfpl1 * (-cos(a1)) + pfpl2 * (-cos(a2)));
            double pLpb1 = -2 * D * (1 - f3 * f4) * \
                (f2 * pf1pl1 / 2 - f1 * pf2pl1 / 2);
            double pLpb2 = -2 * D * (1 - f1 * f2) * \
                (f4 * pf3pl2 / 2 - f3 * pf4pl2 / 2);
            // 累加梯度
            grad += Vec6d(pLpa1, pLpa2, pLpcx, pLpcy, pLpb1, pLpb2);
            // 累加误差
            err = err + D * D;
        }
    }
    // 如果提供了误差指针，则更新误差
    if (err_ptr)
        *err_ptr = err;
    // 返回梯度
    return grad;
}

void fitting(Mat& img, Vec4d& params, int iter_num) {
	CV_Assert(img.type() == CV_8U);
	Vec4d _params = params;
	double err = 0;
	int count = 0;
	while (count < iter_num || iter_num == 0) {
		cout << format("Iterations: %d\n", count);
		cout << format("params: %f %f %f %f\n", _params[0], _params[1], _params[2], _params[3]);
		Vec4d grad = getGrad(img, _params, &err);
		cout << format("err: %f\tgrad: %f %f %f %f\n", err, grad[0], grad[1], grad[2], grad[3]);
		double grad_n = norm(grad);
		if (grad_n < 1e-6)
			break;
		Vec4d grad_cxcy = grad.mul(Vec4d(0, 0, 1, 1));
		Vec4d grad_theta = grad.mul(Vec4d(1, 1, 0, 0));
		double grad_cxcy_n = norm(grad_cxcy);
		double grad_theta_n = norm(grad_theta);
		double step = 0;
		if (grad_cxcy_n > sqrt(grad_theta_n)) {
			if(grad_cxcy_n > 1)
			{
				step = grad_cxcy_n / 500;
			}
			else {
				step = grad_cxcy_n / 500;
			}
			Vec4d d = step * grad_cxcy / grad_cxcy_n;
			_params = _params - d;
			showFunction(img, _params);
			waitKey(100);
			count++;
		}
		else {
			if(grad_theta_n > 1e-2)
			{
				step = grad_theta_n / 60000;
			}
			else {
				step = grad_theta_n / 60000;
			}
			Vec4d d = step * grad_theta / grad_theta_n;
			_params = _params - d;
			showFunction(img, _params);
			waitKey(100);
			count++;
		}
	}
	params = _params;
}

void fitting(Mat& img, Vec6d& params, int iter_num) {
	CV_Assert(img.type() == CV_8U);
	double sum = img.cols * img.rows;
	double err = 0;
	Vec6d _params = params;
	int count = 0;
	while (count < iter_num || iter_num == 0) {
#ifndef CORNERDETECTOR_EXPORTS
#ifdef _DEBUG
		cout << format("Iterations: %d\n", count);
		cout << format("params: %f %f %f %f %f %f\n", _params[0], _params[1], _params[2], _params[3], _params[4], _params[5]);
#endif
#endif
		Vec6d grad = getGrad(img, _params, &err);
		grad /= sum;
#ifndef CORNERDETECTOR_EXPORTS
#ifdef _DEBUG
		cout << format("error: %f\tgrad: %f %f %f %f %f %f\n", err, grad[0], grad[1], grad[2], grad[3], grad[4], grad[5]);
#endif
#endif
		double grad_n = norm(grad);
		if (grad_n < 1e-9)
			break;
		Vec6d grad_cxcy = grad.mul(Vec6d(0, 0, 1, 1, 0, 0));
		Vec6d grad_theta = grad.mul(Vec6d(1, 1, 0, 0, 0, 0));
		Vec6d grad_b1b2 = grad.mul(Vec6d(0, 0, 0, 0, 1, 1));
		double grad_cxcy_n = norm(grad_cxcy);
		double grad_theta_n = norm(grad_theta);
		double grad_b1b2_n = norm(grad_b1b2);
		double step = 0;
		int t = 100;

		if(grad_cxcy_n > 1e-4 || grad_theta_n > 1e-4)
		{
			if (grad_b1b2_n > 100 * grad_cxcy_n || grad_b1b2_n > 100 * grad_theta_n) {
				step = 1.4 * grad_b1b2_n;
				Vec6d d = step * grad_b1b2 / grad_b1b2_n;
				_params = _params - d;
				//showFunction(img, _params);
				//cv::waitKey(t);
				count++;
				continue;
			}
			if (20 * grad_cxcy_n > grad_theta_n) {
				if (grad_cxcy_n > 1e-2) {
					step = grad_cxcy_n/ 1.5;
				}
				else {
					step = grad_cxcy_n;
				}
				Vec6d d = step * grad_cxcy / grad_cxcy_n;
				_params = _params - d;
				//showFunction(img, _params);
				//cv::waitKey(t);
				count++;
			}
			else {
				if (grad_theta_n > 1)
				{
					step = grad_theta_n / 90;
				}
				else {
					step = grad_theta_n / 90;
				}
				Vec6d d = step * grad_theta / grad_theta_n;
				_params = _params - d;
				//showFunction(img, _params);
				//cv::waitKey(t);
				count++;
			}
		}
		else if(grad_cxcy_n>1e-5 ||grad_theta_n>1e-5) {
			if ((grad_b1b2_n > grad_cxcy_n || 100 * grad_b1b2_n > grad_theta_n) && grad_b1b2_n > 1e-4) {
				step = 1.4 * grad_b1b2_n;
				Vec6d d = step * grad_b1b2 / grad_b1b2_n;
				_params = _params - d;
				//showFunction(img, _params);
				//cv::waitKey(t);
				count++;
				continue;
			}
			Vec6d dtheta = grad_theta_n / 80 * grad_theta / grad_theta_n;
			Vec6d dcxcy = 1.2 * grad_cxcy_n * grad_cxcy / grad_cxcy_n;
			Vec6d d = dtheta + dcxcy;
			_params = _params - d;
			//showFunction(img, _params);
			//cv::waitKey(t);
			count++;
		}
		else if(grad_cxcy_n>2e-6 || grad_theta_n>2e-6) {
			Vec6d d;
			if (grad_b1b2_n < grad_cxcy_n || grad_b1b2_n < grad_theta_n)
			{
				Vec6d dtheta = grad_theta_n / 70 * grad_theta / grad_theta_n;
				Vec6d dcxcy = 2.45 * grad_cxcy_n * grad_cxcy / grad_cxcy_n;
				d = dtheta + dcxcy;
			}
			else {
				Vec6d dtheta = grad_theta_n / 70 * grad_theta / grad_theta_n;
				Vec6d dcxcy = 2.45 * grad_cxcy_n * grad_cxcy / grad_cxcy_n;
				Vec6d db1b2 = 2.8 * grad_b1b2_n * grad_b1b2 / grad_b1b2_n;
				d = dtheta + dcxcy + db1b2;
			}
			_params = _params - d;
			//showFunction(img, _params);
			//cv::waitKey(t);
			count++;
		}
		else {
			Vec6d d;
			if (grad_b1b2_n < grad_cxcy_n || grad_b1b2_n < grad_theta_n)
			{
				Vec6d dtheta = grad_theta_n / 70 * grad_theta / grad_theta_n;
				Vec6d dcxcy = 2.54 * grad_cxcy_n * grad_cxcy / grad_cxcy_n;
				d = dtheta + dcxcy;
			}
			else {
				Vec6d dtheta = grad_theta_n / 70 * grad_theta / grad_theta_n;
				Vec6d dcxcy = 2.54 * grad_cxcy_n * grad_cxcy / grad_cxcy_n;
				Vec6d db1b2 = 2.8 * grad_b1b2_n * grad_b1b2 / grad_b1b2_n;
				d = dtheta + dcxcy + db1b2;
			}
			_params = _params - d;
			//showFunction(img, _params);
			//cv::waitKey(t);
			count++;
		}
	}
	params = _params;
}

void fitting(Mat& img, Vec6d& params, double learningRate, int iter_num) {
	CV_Assert(img.type() == CV_8U);
	double sum = img.cols * img.rows;
	double err = 0;
	Vec6d _params = params;
	int count = 0;
	while (count < iter_num || iter_num == 0) {
#ifndef CORNERDETECTOR_EXPORTS
#ifdef _DEBUG
		cout << format("Iterations: %d\n", count);
		cout << format("params: %f %f %f %f %f %f\n", _params[0], _params[1], _params[2], _params[3], _params[4], _params[5]);
#endif
#endif
		Vec6d grad = getGrad(img, _params, &err);
		grad /= sum;
#ifndef CORNERDETECTOR_EXPORTS
#ifdef _DEBUG
		cout << format("error: %f\tgrad: %f %f %f %f %f %f\n", err, grad[0], grad[1], grad[2], grad[3], grad[4], grad[5]);
#endif
#endif
		double grad_n = norm(grad);
		if (grad_n < 1e-6)
			break;
		Vec6d d = learningRate * grad;
		_params = _params - d;
		count++;
	}
	params = _params;

}

void fitting_newton(Mat& img, Vec4d& params, int iter_num) {
	CV_Assert(img.type() == CV_8U);
	Vec4d _params = params;
	int count = 0;
	Vec4d prev_grad;
	Vec4d prev_d;
	while (count < iter_num || iter_num == 0) {
		cout << format("Iterations: %d\n", count);
		cout << format("params: %f %f %f %f\n", _params[0], _params[1], _params[2], _params[3]);
		Vec4d grad = getGrad(img, _params);
		double grad_n = norm(grad);
		if (grad_n < 1e-6)
			break;
		Vec4d grad_cxcy = grad.mul(Vec4d(0, 0, 1, 1));
		Vec4d grad_theta = grad.mul(Vec4d(1, 1, 0, 0));
		double grad_cxcy_n = norm(grad_cxcy);
		double grad_theta_n = norm(grad_theta);
		Vec4d dgrad = prev_grad - grad;
		if (grad_cxcy_n > sqrt(grad_theta_n)) {
			Vec4d d(0, 0, 0, 0);
			if(count!=0)
			{
				Matx<double, 2, 1> g;
				g << grad[2], grad[3];
				Matx<double, 2, 2> gg;
				gg << dgrad[2] / prev_d[2], dgrad[2] / prev_d[3], dgrad[3] / prev_d[2], dgrad[3] / prev_d[3];
				Matx<double, 2, 1> dMat = gg.inv() * g;
				d = 0.001 * Vec4d(0, 0, dMat(0), dMat(1)).mul(grad_cxcy) / grad_cxcy_n;
			}
			else {
				double step = grad_cxcy_n / 350;
				d = step * grad_cxcy / grad_cxcy_n;
			}
			_params = _params - d;
			prev_d = d;
			showFunction(img, _params);
			waitKey(300);
			count++;
		}
		else {
			Vec4d d(0, 0, 0, 0);
			if(count!=0)
			{
				Matx<double, 2, 1> g;
				g << grad[0], grad[1];
				Matx<double, 2, 2> gg;
				gg << dgrad[0] / prev_d[0], dgrad[0] / prev_d[1], dgrad[1] / prev_d[0], dgrad[1] / prev_d[1];
				Matx<double, 2, 1> dMat = gg.inv() * g;
				d = 0.001 * Vec4d(dMat(0), dMat(1), 0, 0).mul(grad_theta) / grad_theta_n;
			}
			else {
				double step = grad_theta_n / 30000;
				d = step * grad_theta / grad_theta_n;
			}
			_params = _params - d;
			showFunction(img, _params);
			waitKey(300);
			count++;
		}
		prev_grad = grad;
	}
	params = _params;

}

void showFunction(Mat img, Vec4d params) {
	//double a1 = params[0], a2 = params[3], r1 = params[1], r2 = params[4], b1 = params[2], b2 = params[5];
	double a1 = params[0], a2 = params[1], cx = params[2], cy = params[3];
	Mat canva;
	cvtColor(img, canva, COLOR_GRAY2BGR);
	for (int i = 0; i < canva.rows; i++) {
		Vec3b* data = canva.ptr<Vec3b>(i);
		for (int j = 0; j < canva.cols; j++) {
			Point2d coord(j, i);
			Point2d coeff1(cos(a1), sin(a1));
			Point2d coeff2(cos(a2), sin(a2));
			double l1 = coeff1.cross(coord) - coeff1.cross(Point2d(cx, cy));
			double l2 = coeff2.cross(coord) - coeff2.cross(Point2d(cx, cy));
			double g1 = exp(-10 * (l1 + 1.5));
			double g2 = exp(10 * (l1 - 1.5));
			double g3 = exp(-10 * (l2 + 1.5));
			double g4 = exp(10 * (l2 - 1.5));
			double f1 = 1 / (1 + g1);
			double f2 = 1 / (1 + g2);
			double f3 = 1 / (1 + g3);
			double f4 = 1 / (1 + g4);
			double f = f1 * f2 + f3 * f4 - f1 * f2 * f3 * f4;
			data[j] = 0.3 * data[j] + 0.7* f * Vec3b(0, 255, 0);
		}
	}
	namedWindow("fitting result", WINDOW_NORMAL);
	imshow("fitting result", canva);
}

void showFunction(Mat img, Vec6d params) {
	//double a1 = params[0], a2 = params[3], r1 = params[1], r2 = params[4], b1 = params[2], b2 = params[5];
	double a1 = params[0], a2 = params[1], cx = params[2], cy = params[3], b1 = params[4], b2 = params[5];
	Mat canva;
	cvtColor(img, canva, COLOR_GRAY2BGR);
	for (int i = 0; i < canva.rows; i++) {
		Vec3b* data = canva.ptr<Vec3b>(i);
		for (int j = 0; j < canva.cols; j++) {
			Point2d coord(j, i);
			Point2d coeff1(cos(a1), sin(a1));
			Point2d coeff2(cos(a2), sin(a2));
			double l1 = coeff1.cross(coord) - coeff1.cross(Point2d(cx, cy));
			double l2 = coeff2.cross(coord) - coeff2.cross(Point2d(cx, cy));
			double g1 = exp(-10 * (l1 + b1/2));
			double g2 = exp(10 * (l1 - b1/2));
			double g3 = exp(-10 * (l2 + b2/2));
			double g4 = exp(10 * (l2 - b2/2));
			double f1 = 1 / (1 + g1);
			double f2 = 1 / (1 + g2);
			double f3 = 1 / (1 + g3);
			double f4 = 1 / (1 + g4);
			double f = f1 * f2 + f3 * f4 - f1 * f2 * f3 * f4;
			data[j] = 0.3 * data[j] + 0.7 * f * Vec3b(0, 255, 0);
		}
	}
	
	double scale = 40;
	//double xScale = (img.cols * scale + 1) / (img.cols + 1);
	//double yScale = (img.rows * scale + 1) / (img.rows + 1);
	//printf("cx: %f  cy: %f  rows: %d  cols: %d  ", cx, cy, canva.rows, canva.cols);
	resize(canva, canva, Size(), scale, scale, INTER_NEAREST);
	cx += 0.5;
	cy += 0.5;
	cx *= scale;
	cy *= scale;
	//Point2d l1p1(max(cx - cy / tan(a1), 0.), max(cy - tan(a1) * cx, 0.)),
	//	l1p2(min(cx + (canva.rows + 1 - cy) / tan(a1), double(canva.cols)), min(cy + tan(a1) * (canva.cols + 1 - cx), double(canva.rows))),
	//	l2p1(max(cx + (canva.rows + 1 - cy) / tan(a2), 0.), min(cy - tan(a2) * cx, double(canva.rows))),
	//	l2p2(min(cx - cy / tan(a2), double(canva.cols)), max(cy + tan(a2) * (canva.cols + 1 - cx), 0.));
	//line(canva, l1p1, l1p2, Scalar(0, 0, 255), 1);
	//line(canva, l2p1, l2p2, Scalar(0, 0, 255), 1);
	drawLine(canva, Scalar(0, 0, 255), -sin(a1), cos(a1), cx * sin(a1) - cy * cos(a1), 1);
	drawLine(canva, Scalar(0, 0, 255), -sin(a2), cos(a2), cx * sin(a2) - cy * cos(a2), 1);
	circle(canva, Point(cx, cy), 5, Scalar());
	//printf("cx: %f  cy: %f  rows: %d  cols: %d\n", cx, cy, canva.rows, canva.cols);
	namedWindow("fitting result", WINDOW_NORMAL);
	imshow("fitting result", canva);
	int key = waitKey();
	if (key == 's') {
		string filename;
		cout << "Please input the file name:\n";
		cin >> filename;
		imwrite(filename, canva);
	}

}

// 函数：showLines
// 功能：在给定的图像上绘制两条直线和一个圆点
// 参数：
//   canva - 用于绘制的图像
//   params - 包含直线和圆点参数的向量，格式为[a1, a2, cx, cy, b1, b2]
//   scale - 缩放比例
//   color - 绘制颜色
void showLines(Mat& canva, Vec6d params, double scale, Scalar color) {
    // 提取参数
    double a1 = params[0], a2 = params[1], cx = params[2], cy = params[3], b1 = params[4], b2 = params[5];
    
    // 将中心点坐标调整为图像中心
    cx += 0.5;
    cy += 0.5;
    
    // 根据缩放比例调整中心点坐标
    cx *= scale;
    cy *= scale;
    
    // 计算第一条直线的两个端点
    Point2d l1p1(max(cx - cy / tan(a1), 0.), max(cy - tan(a1) * cx, 0.)),
           l1p2(min(cx + (canva.rows + 1 - cy) / tan(a1), double(canva.cols)), 
                min(cy + tan(a1) * (canva.cols + 1 - cx), double(canva.rows)));
                
    // 计算第二条直线的两个端点
    Point2d l2p1(max(cx + (canva.rows + 1 - cy) / tan(a2), 0.), 
                 min(cy - tan(a2) * cx, double(canva.rows))),
           l2p2(min(cx - cy / tan(a2), double(canva.cols)), 
                max(cy + tan(a2) * (canva.cols + 1 - cx), 0.));
                
    // 绘制第一条直线
    line(canva, l1p1, l1p2, color, 2);
    
    // 绘制第二条直线
    line(canva, l2p1, l2p2, color, 2);
    
    // 绘制中心点
    circle(canva, Point(cx, cy), 5, Scalar());
}

// 函数：showFunction_lines
// 功能：在给定的图像上绘制两条直线和一个圆点，并显示结果
// 参数：
//   img - 输入的灰度图像
//   params - 包含直线和圆点参数的向量，格式为[a1, a2, cx, cy, b1, b2]
//   scale - 缩放比例
void showFunction_lines(Mat img, Vec6d params, double scale) {
    // 提取参数
    double a1 = params[0], a2 = params[1], cx = params[2], cy = params[3], b1 = params[4], b2 = params[5];
    
    // 创建用于绘制的图像
    Mat canva;
    
    // 将灰度图像转换为彩色图像
    cvtColor(img, canva, COLOR_GRAY2BGR);
    
    // 根据缩放比例调整图像大小
    resize(canva, canva, Size(), scale, scale, INTER_NEAREST);
    
    // 调整中心点坐标
    cx += 0.5;
    cy += 0.5;
    cx *= scale;
    cy *= scale;
    
    // 调整偏移量
    b1 *= scale;
    b2 *= scale;
    
    // 绘制第一条直线的两条平行线
    drawLine(canva, Scalar(255, 0, 0), -sin(a1), cos(a1), cx * sin(a1) - cy * cos(a1), 3);
    drawLine(canva, Scalar(255, 0, 0), -sin(a1), cos(a1), cx * sin(a1) - cy * cos(a1) + b1 / 2, 3);
    drawLine(canva, Scalar(255, 0, 0), -sin(a1), cos(a1), cx * sin(a1) - cy * cos(a1) - b1 / 2, 3);
    
    // 绘制第二条直线的两条平行线
    drawLine(canva, Scalar(255, 0, 0), -sin(a2), cos(a2), cx * sin(a2) - cy * cos(a2), 3);
    drawLine(canva, Scalar(255, 0, 0), -sin(a2), cos(a2), cx * sin(a2) - cy * cos(a2) + b2 / 2, 3);
    drawLine(canva, Scalar(255, 0, 0), -sin(a2), cos(a2), cx * sin(a2) - cy * cos(a2) - b2 / 2, 3);
    
    // 绘制中心点
    circle(canva, Point(cx, cy), 5, Scalar());
    
    // 创建窗口并显示结果
    namedWindow("fitting result", WINDOW_NORMAL);
    imshow("fitting result", canva);
    
    // 等待按键
    int key = waitKey();
    
    // 如果按下's'键，保存图像
    if (key == 's') {
        string filename;
        cout << "Please input the file name:\n";
        cin >> filename;
        imwrite(filename, canva);
    }
}