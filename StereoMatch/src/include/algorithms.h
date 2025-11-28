#pragma once

#include "opencv2/imgcodecs.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/ximgproc.hpp"
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
    
    // WLS滤波器参数
    bool useWLS = true;
    double wlsLambda = 8000.0;    // WLS平滑参数
    double wlsSigma = 1.5;        // WLS颜色相似性参数
    
    // Fast Bilateral Solver参数
    bool useFBS = true;          // 是否使用FBS
    double fbsLambda = 128.0;     // FBS平滑参数
    double fbsSigma = 0.05;       // FBS颜色相似性参数
    
    // WLS参数设置函数
    void setUseWLS(bool use) { useWLS = use; }
    void setWLSLambda(double val) { wlsLambda = val; }
    void setWLSSigma(double val) { wlsSigma = val; }
    bool getUseWLS() const { return useWLS; }
    double getWLSLambda() const { return wlsLambda; }
    double getWLSSigma() const { return wlsSigma; }
    
    // FBS参数设置函数
    void setUseFBS(bool use) { useFBS = use; }
    void setFBSLambda(double val) { fbsLambda = val; }
    void setFBSSigma(double val) { fbsSigma = val; }
    bool getUseFBS() const { return useFBS; }
    double getFBSLambda() const { return fbsLambda; }
    double getFBSSigma() const { return fbsSigma; }
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

