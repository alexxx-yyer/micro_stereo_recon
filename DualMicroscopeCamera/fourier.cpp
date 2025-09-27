#include "fourier.h"  // 包含自定义的头文件，可能包含一些FFT相关的函数和定义
#include <iostream>   // 包含标准输入输出流库

using namespace cv;  // 使用OpenCV命名空间
using namespace std; // 使用标准命名空间

// 函数：fftshift
// 功能：对输入的图像进行FFT变换后的频谱图像的象限交换
// 参数：inputImg - 输入的图像矩阵
void fftshift(Mat inputImg) {
    assert(inputImg.rows % 2 == 0); // 断言：图像行数必须为偶数
    assert(inputImg.cols % 2 == 0); // 断言：图像列数必须为偶数
    int cx = inputImg.cols / 2;     // 计算图像中心点的x坐标
    int cy = inputImg.rows / 2;     // 计算图像中心点的y坐标

    // 定义四个象限的子矩阵
    Mat p0(inputImg, Rect(0, 0, cx, cy)),   // 第一象限
        p1(inputImg, Rect(cx, 0, cx, cy)),  // 第二象限
        p2(inputImg, Rect(0, cy, cx, cy)),  // 第三象限
        p3(inputImg, Rect(cx, cy, cx, cy)); // 第四象限

    Mat tmp; // 临时矩阵用于交换数据
    p0.copyTo(tmp); // 将第一象限数据复制到临时矩阵
    p3.copyTo(p0);  // 将第四象限数据复制到第一象限
    tmp.copyTo(p3); // 将临时矩阵数据复制到第四象限
    p1.copyTo(tmp); // 将第二象限数据复制到临时矩阵
    p2.copyTo(p1);  // 将第三象限数据复制到第二象限
    tmp.copyTo(p2); // 将临时矩阵数据复制到第三象限
}

// 函数：fourierTransform
// 功能：对输入图像进行傅里叶变换
// 参数：inputImg - 输入的图像矩阵
// 返回：指向变换结果的指针
Mat* fourierTransform(Mat inputImg) {
    Mat* output = new Mat(inputImg.size(), CV_32FC2); // 创建输出矩阵，大小与输入相同，类型为32位浮点复数
    Mat planes[2] = { Mat_<float>(inputImg),Mat::zeros(inputImg.size(),CV_32F) }; // 创建两个平面，一个存放输入图像，一个为全零矩阵
    merge(planes, 2, *output); // 将两个平面合并为一个复数矩阵
    dft(*output, *output);     // 对合并后的矩阵进行傅里叶变换
    return output;             // 返回变换结果的指针
}

// 函数：showFT
// 功能：显示傅里叶变换后的频谱图像
// 参数：inputFT - 输入的傅里叶变换结果矩阵
void showFT(Mat inputFT) {
    Mat planes[2];             // 定义两个平面用于存放实部和虚部
    split(inputFT,planes);     // 将输入的复数矩阵拆分为实部和虚部
    Mat FTimg(inputFT.size(), CV_32F); // 创建频谱图像矩阵
    magnitude(planes[0], planes[1], FTimg); // 计算频谱幅度
    FTimg += 1;                // 频谱幅度加1，避免对数计算时出现负值
    log(FTimg, FTimg);         // 对频谱幅度取对数
    normalize(FTimg, FTimg, 0, 1, NORM_MINMAX); // 归一化处理，使频谱幅度在0到1之间
    imshow("FTimg", FTimg);    // 显示频谱图像
}

// 函数：hpFilter
// 功能：生成高通滤波器
// 参数：size - 滤波器大小，radius - 滤波器半径
// 返回：指向滤波器矩阵的指针
Mat* hpFilter(Size size, int radius) {
    Mat filter(size, CV_32F); // 创建滤波器矩阵，类型为32位浮点数
    int cx = size.width / 2;  // 计算滤波器中心点的x坐标
    int cy = size.height / 2; // 计算滤波器中心点的y坐标
    int r = radius * radius;  // 计算半径的平方

    // 遍历滤波器矩阵的每个像素
    for (int i = 0; i < size.height; i++) {
        float* ptr = (float*)(filter.ptr(i)); // 获取当前行的指针
        for (int j = 0; j < size.width; j++) {
            // 判断当前像素是否在半径内
            if (pow(j - cx, 2) + pow(i - cy, 2) <= r)
                ptr[j] = 0; // 半径内的像素值设为0
            else
                ptr[j] = 1; // 半径外的像素值设为1
        }
    }
    Mat planes[2] = { filter,filter }; // 创建两个相同的平面
    Mat* output = new Mat(size, CV_32FC2); // 创建输出矩阵，类型为32位浮点复数
    merge(planes, 2, *output); // 将两个平面合并为一个复数矩阵
    return output;             // 返回滤波器矩阵的指针
}