#include "opencv2/imgcodecs.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <filesystem>
#include "StereoMatchingTools.h"
#include "DirTools.h"

using namespace std;
using namespace cv;

namespace fsys = filesystem;

const string keys = 
    "{@input1||left path}"
    "{intrinsics||camera instrinsics file}"
    "{extrinsics||camera extrinsics file}"
    "{spath||saved directory}";

int main(int argc, char* argv[]) {
    CommandLineParser parser(argc, argv, keys);
    string lpath = parser.get<string>(0);
    if(lpath.empty()) {
        cout << "Please enter the image pair list file:\n";
        cin >> lpath;
    }
    string intrin = parser.get<string>("intrinsics");
    string extrin = parser.get<string>("extrinsics");
    if(intrin.empty()) {
        cout << "Please enter the camera intrinsics file path:\n";
        cin >> intrin;
    }
    if(extrin.empty()) {
        cout << "Please enter the camera extrinsics file path:\n";
        cin >> extrin;
    }
    string spath = parser.get<string>("spath");
    if(spath.empty()) {
        cout << "Please enter the directory for saving images\n";
        cin >> spath;
    }
    vector<string> lImgs, rImgs;
    loadImgPair_YML(fsys::path(lpath), lImgs, rImgs);
    CamParams params;
    if (!loadCamParams(params, intrin, extrin)) {
        cerr << "load camera parameters failed\n";
    }
    for(int i = 0; i<lImgs.size(); i++) {
        Mat imgL = imread(lImgs[i]);
        Mat imgR = imread(rImgs[i]);
        printf("load image:%s\n", lImgs[i].c_str());
        printf("load image:%s\n", rImgs[i].c_str());
        Mat r_imgL, r_imgR;
        rectifyImagePair(imgL, imgR, params, r_imgL, r_imgR);
        fsys::path left = fsys::path(spath) / fsys::path("left") / fsys::path(lImgs[i]).stem();
        fsys::path right = fsys::path(spath) / fsys::path("right") / fsys::path(rImgs[i]).stem();
        if(!fsys::exists(left.parent_path())) {
            fsys::create_directories(left.parent_path());
        }
        if(!fsys::exists(right.parent_path())) {
            fsys::create_directories(right.parent_path());
        }
        imwrite(left.string() + "_rectified.png", r_imgL);
        imwrite(right.string() + "_rectified.png", r_imgR);
    }
    return 1;
}