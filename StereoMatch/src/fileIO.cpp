#include "fileIO.h"
#include <filesystem>
#include "opencv2/imgcodecs.hpp"
#include "pcl/io/pcd_io.h"

using namespace std;
using namespace cv;
namespace fsys = filesystem;

void saveDisp(cv::Mat& disp, std::string dir, std::string path) {
    fsys::path name = fsys::path(path).stem();
    name += "_disp.tiff";
    fsys::path spath = fsys::path(dir) / fsys::path(name);
    imwrite(spath.string(), disp);
}

void saveDisp8(cv::Mat& disp8, std::string dir, std::string path) {
    fsys::path name = fsys::path(path).stem();
    name += "_disp8.png";
    fsys::path spath = fsys::path(dir) / fsys::path(name);
    imwrite(spath.string(), disp8);
}

void xyz2cloud(cv::Mat* xyz, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    cloud->clear();
    Vec3f* point = xyz->ptr<Vec3f>(0);
    for (size_t i = 0; i < xyz->total(); i++) {
        pcl::PointXYZ p(point[i][0], point[i][1], point[i][2]);
        cloud->push_back(p);
    }
}

void saveCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string dir, std::string path) {
    fsys::path name = fsys::path(path).stem();
    name += "_cloud.pcd";
    fsys::path spath = fsys::path(dir) / fsys::path(name);
    pcl::io::savePCDFileBinary(spath.string(), *cloud);
}