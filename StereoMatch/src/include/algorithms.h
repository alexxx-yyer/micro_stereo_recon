#pragma once

#include "opencv2/imgcodecs.hpp"
#include "opencv2/calib3d.hpp"
#include "elas/elasLib.h"

class Alg_Impl {
public:
    virtual ~Alg_Impl() {};
    virtual cv::Mat match(cv::Mat &lImg, cv::Mat &rImg) = 0;
    virtual std::string getType() = 0;
    virtual int getDispNum() = 0;
};

class Alg_SGBM_Impl : public Alg_Impl {
public:
    cv::Mat match(cv::Mat &lImg, cv::Mat &rImg);
    std::string getType() {return "SGBM";}
    int getDispNum();

public:
    Alg_SGBM_Impl();

    cv::Ptr<cv::StereoSGBM> alg = nullptr;
};

class Alg_BM_Impl : public Alg_Impl {
public:
    cv::Mat match(cv::Mat &lImg, cv::Mat &rImg);
    std::string getType() {return "BM";}
    int getDispNum();

public:
    Alg_BM_Impl();

    cv::Ptr<cv::StereoBM> alg = nullptr;
};

class Alg_ELAS_Impl : public Alg_Impl {
public:
    cv::Mat match(cv::Mat &lImg, cv::Mat &rImg);
    std::string getType() {return "ELAS";}
    int getDispNum();

public:
    Alg_ELAS_Impl();
    ~Alg_ELAS_Impl();

    elas_ocv *alg=nullptr;
};

