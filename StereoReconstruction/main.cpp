#include "OCV_PCL.h"
#include "StereoMatchingTools.h"
#include <iostream>
#include <filesystem>

#include "opencv2/imgcodecs.hpp"
#include "pcl/io/ply_io.h"

using namespace std;
using namespace cv;
using namespace pcl;

namespace fsys = filesystem;

const string keys = 
    "{@disp||disparity file path}"
    "{@img||rgb image path}"
    "{@QFile||Q mat file path}";

int main(int argc, char* argv[]) {
    CommandLineParser parser(argc, argv, keys);
    string disp_path = parser.get<string>(0);
    string img_path = parser.get<string>(1);
    string Q_path = parser.get<string>(2);
    if (disp_path.empty()) {
        cout << "Please enter the disparity file path:\n";
        cin >> disp_path;
    }
    if (img_path.empty()) {
        cout << "Please enter the RGB image file path:\n";
        cin >> img_path;
    }
    if (Q_path.empty()) {
        cout << "Please enter the Q file path: (commonly the extrinsics file)\n";
        cin >> Q_path;
    }
    Mat disp = imread(disp_path, IMREAD_UNCHANGED);
    Mat img = imread(img_path);
    Mat Q = loadQMat(Q_path);
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    disp2cloud_rgb(&disp, &Q, &img, cloud);
    fsys::path file_name = fsys::path(img_path).stem();
    string spath;
    cout << "Please enter the directory for saving point cloud:\n";
    cin >> spath;
    file_name = fsys::path(spath) / file_name;
    io::savePLYFileBinary(file_name.string() + ".ply", *cloud);
    return 0;
}