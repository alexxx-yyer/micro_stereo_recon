#include "algorithms.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>

using namespace cv;
using namespace std;

Alg_SGBM_Impl::Alg_SGBM_Impl() {
    alg = StereoSGBM::create();
}

Mat Alg_SGBM_Impl::match(Mat &lImg, Mat &rImg) {
    Mat disp;
    alg->compute(lImg, rImg, disp);
    Mat res(disp.size(), CV_32F);
    disp.convertTo(res, CV_32F, 1/16.);
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