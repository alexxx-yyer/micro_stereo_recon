// 输入：极线校正后的左右视图
// 输出：视差图

#include <iostream>

#include "fileIO.h"
#include "interactive.h"
#include "algorithms.h"

#include "VisualizationTools.h"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

const string keys = 
"{@input1||images}"
"{@input2||right images}"
"{intrinsics||camera intrinsics}"
"{extrinsics||camera extrinsics}"
"{algorithm|SGBM|specifiy the stereo match algorithm}";

int main(int argc, char* argv[]) {
    CommandLineParser parser(argc, argv, keys);
    CommandLineEngine engine(parser);
    return engine.run();    
}