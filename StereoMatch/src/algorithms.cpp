#include "algorithms.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/ximgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

Alg_SGBM_Impl::Alg_SGBM_Impl() {
    alg = StereoSGBM::create();
    
    // 设置SGBM的默认参数
    alg->setBlockSize(5);           // 块大小
    alg->setNumDisparities(256);     // 视差数量
    alg->setMinDisparity(0);        // 最小视差
    alg->setP1(400);          // P1参数
    alg->setP2(3200);         // P2参数
    alg->setPreFilterCap(63);       // 预滤波截断值
    alg->setUniquenessRatio(0);    // 唯一性比率
    alg->setSpeckleWindowSize(0); // 斑点窗口大小
    alg->setSpeckleRange(0);       // 斑点范围
    alg->setDisp12MaxDiff(1);       // 左右视差最大差异
    alg->setMode(StereoSGBM::MODE_SGBM); // 模式
    
    // 设置后处理滤波器的默认参数
    useWLS = true;                   // 默认使用WLS滤波器
    useFBS = true;                  // 默认不使用FBS
    wlsLambda = 8000.0;             // WLS平滑参数
    wlsSigma = 1.5;                 // WLS颜色相似性参数
    fbsLambda = 128.0;              // FBS平滑参数
    fbsSigma = 0.05;                // FBS颜色相似性参数
}

Mat Alg_SGBM_Impl::match(Mat &lImg, Mat &rImg) {
    Mat disp_left, disp_right, disp_wls, disp;

    // 基础 SGBM 计算
    alg->compute(lImg, rImg, disp_left);

    if (useWLS || useFBS) {
        try {
            // 先进行WLS
            if (useWLS) {
                Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(alg);
                right_matcher->compute(rImg, lImg, disp_right);

                Ptr<ximgproc::DisparityWLSFilter> wls_filter =
                    ximgproc::createDisparityWLSFilter(alg);
                wls_filter->setLambda(wlsLambda);
                wls_filter->setSigmaColor(wlsSigma);

                wls_filter->filter(disp_left, lImg, disp_wls, disp_right);
            } else {
                disp_wls = disp_left;
            }

            // 再进行FBS
            if (useFBS) {
                Ptr<ximgproc::FastBilateralSolverFilter> fbs_filter =
                    ximgproc::createFastBilateralSolverFilter(lImg, fbsSigma, fbsSigma, fbsSigma, fbsLambda);

                Mat confidence = Mat::ones(disp_wls.size(), CV_32F);
                fbs_filter->filter(disp_wls, confidence, disp);
            } else {
                disp = disp_wls;
            }
        } catch (const cv::Exception& e) {
            std::cerr << "Post filter error: " << e.what() << std::endl;
            disp = disp_left;
        }
    } else {
        disp = disp_left;
    }

    // 转换格式
    Mat res(disp.size(), CV_32F);
    disp.convertTo(res, CV_32F, 1 / 16.0);
    return res;
}

int Alg_SGBM_Impl::getDispNum() {
    return alg->getNumDisparities();
}

/********************************************************************/
/*                         BM Algorithm                             */
/********************************************************************/

Alg_BM_Impl::Alg_BM_Impl() {
    alg = StereoBM::create();
}

Mat Alg_BM_Impl::match(Mat &lImg, Mat &rImg) {
    Mat disp;
    alg->compute(lImg, rImg, disp);
    Mat res(disp.size(), CV_32F);
    disp.convertTo(res, CV_32F, 1/16.);
    return res;
}

int Alg_BM_Impl::getDispNum() {
    return alg->getNumDisparities();
}

/********************************************************************/
/*                       ELAS Algorithm                             */
/********************************************************************/

Alg_ELAS_Impl::Alg_ELAS_Impl() : alg(new elas_ocv()) {}

Alg_ELAS_Impl::~Alg_ELAS_Impl() {delete alg;}

Mat Alg_ELAS_Impl::match(Mat &lImg, Mat &rImg) {
    Mat dispL(lImg.size(), CV_32F);
    Mat dispR(rImg.size(), CV_32F);
    alg->match(&lImg, &rImg, &dispL, &dispR);
    return dispL;
}

int Alg_ELAS_Impl::getDispNum() {
    int dispNum = 1 + alg->elas->get_disp_max() - alg->elas->get_disp_min();
    return dispNum;
}