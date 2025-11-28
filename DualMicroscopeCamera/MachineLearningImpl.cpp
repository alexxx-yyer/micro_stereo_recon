#include "MachineLearningImpl.h"  // 包含机器学习实现的头文件

#include "opencv2/core.hpp"  // 包含OpenCV核心功能头文件

#include <iostream>  // 包含标准输入输出流头文件

using namespace std;  // 使用标准命名空间
using namespace cv;  // 使用OpenCV命名空间

// 生成随机样本对的函数
// 参数range表示样本范围，total表示需要生成的样本对数量，默认为10
static vector<pair<size_t, size_t>>  random_sample(size_t range, int total = 10) {
    RNG rng(time(nullptr));  // 使用当前时间初始化随机数生成器
    vector<pair<size_t, size_t>> res;  // 存储生成的样本对
    int count = 0;  // 计数器，记录已生成的样本对数量
    while(count < total) {  // 循环直到生成足够的样本对
        size_t first = rng(range);  // 生成第一个随机样本索引
        size_t second = rng(range);  // 生成第二个随机样本索引
        if (first != second) {  // 确保两个索引不同
            res.emplace_back(first, second);  // 将样本对添加到结果中
            count++;  // 增加计数器
        }
    }
    return res;  // 返回生成的样本对
}

// 计算最佳得分并找到内点集的函数
// 参数Dataset为数据集，idxs为样本对索引，threshold为阈值，inlier为输出内点集
static int bestScore(vector<Point>& Dataset, vector<pair<size_t, size_t>> idxs, double threshold, vector<Point>& inlier) {
    int _res = 0;  // 初始化最佳得分为0
    vector<Point> _inlier;  // 存储当前最佳内点集
    for(int i=0;i<idxs.size();i++)  // 遍历所有样本对
    {
        int score = 0;  // 初始化当前样本对的得分为0
        vector<Point> _inlier_candidates;  // 存储当前样本对的内点候选集
        Point p1 = Dataset[idxs[i].first];  // 获取第一个样本点
        Point p2 = Dataset[idxs[i].second];  // 获取第二个样本点
        double a = (p2 - p1).y;  // 计算直线方程的a系数
        double b = (p1 - p2).x;  // 计算直线方程的b系数
        double factor = sqrt(a * a + b * b);  // 计算a和b的模
        a /= factor;  // 归一化a
        b /= factor;  // 归一化b
        double c = -(a * p1.x + b * p1.y);  // 计算直线方程的c系数
        for (size_t k = 0; k < Dataset.size(); k++) {  // 遍历所有数据点
            if (fabs(a * Dataset[k].x + b * Dataset[k].y + c) < threshold)  // 判断点是否在直线上
            {
                score += 1;  // 增加当前样本对的得分
                _inlier_candidates.push_back(Dataset[k]);  // 将点添加到内点候选集中
            }
        }
        if (score > _res) {  // 如果当前样本对的得分高于最佳得分
            _res = score;  // 更新最佳得分
            _inlier = _inlier_candidates;  // 更新最佳内点集
        }
    }
    inlier = _inlier;  // 输出最佳内点集
    return _res;  // 返回最佳得分
}

// 使用RANSAC算法拟合直线的函数
// 参数Dataset为数据集，inlier为输出内点集
void fitLineRANSAC(vector<Point>& Dataset, vector<Point>& inlier) {
    size_t range = Dataset.size();  // 获取数据集大小
    vector<pair<size_t, size_t>> sample = random_sample(range, 15);  // 生成15个随机样本对
    int score = bestScore(Dataset, sample, 2, inlier);  // 计算最佳得分并找到内点集
    cout << format("%d", score);  // 输出最佳得分
}