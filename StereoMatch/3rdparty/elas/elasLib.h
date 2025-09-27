#pragma once

#include "opencv2/core.hpp"
#include "elas.h"

void elas_cv(cv::Mat* imgL, cv::Mat* imgR, cv::Mat* dispL, cv::Mat* dispR);

class elas_ocv {
public:
    elas_ocv();
    ~elas_ocv();

    void match(cv::Mat* imgL, cv::Mat* imgR, cv::Mat* dispL, cv::Mat* dispR);
    Elas *elas;
};