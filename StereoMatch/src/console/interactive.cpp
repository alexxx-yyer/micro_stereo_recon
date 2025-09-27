#include <iostream>

#include "interactive.h"
#include "fileIO.h"

#include "VisualizationTools.h"
#include "StereoMatchingTools.h"
#include "OCV_PCL.h"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include "pcl/io/pcd_io.h"

using namespace cv;
using namespace std;
using namespace pcl;
namespace fsys = filesystem;

string outputDir = "";
bool isSaveCloud = false;

step CommandLineEngine::process(string lpath, string rpath) {
    Alg_Impl *alg_ptr = alg->algorithm_ptr;
    Mat lImg = imread(lpath);
    Mat rImg = imread(rpath);
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    dualView(lImg, rImg, "Rectified Image Pair");
    while(1) {
        Mat disp = alg_ptr->match(lImg, rImg);
        Mat disp8 = showDisp(&disp, alg_ptr->getDispNum(), "Disparity");
        int key = waitKey();
        if (key == 'q')
            return step::Exit;
        if (key == 'c') {
            alg->modifySetting();
            cloud->clear();
        }
        else if (key == 'p')
            alg->showSetting();
        else if (key == 's') {
            if (outputDir == string()) {
                cout << "Please enter the directory to store result:\n";
                cin >> outputDir;
            }
            saveDisp(disp, outputDir, lpath);
            saveDisp8(disp8, outputDir, lpath);
            if (isSaveCloud) {
                if (Q.empty()) {
                    cout << "Please enter the Q mat file path:\n";
                    string Qpath;
                    cin >> Qpath;
                    Q = loadQMat(Qpath);
                }
                if (cloud->empty())
                    disp2cloud_rgb(&disp, &Q, &lImg, cloud);
                saveCloud(cloud, outputDir, lpath);
            }
        }
        else if (key == 'v') {
            if (Q.empty()) {
                cout << "Please enter the Q mat file path:\n";
                string Qpath;
                cin >> Qpath;
                Q = loadQMat(Qpath);
            }
            if (cloud->empty())
                disp2cloud_rgb(&disp, &Q, &lImg, cloud);
            viewCloud(cloud);
        }
        else if (key == '.')
            return GoNext;
        else if (key == ',')
            return GoBack;
    }
}

bool CommandLineEngine::getImageList(std::vector<std::string> &lImgs, std::vector<std::string> &rImgs) {
    string input = parser.get<string>(0);
    if (input == string()) {
        cout << "Please enter the directory or the path of left image. "
            "Alternatively, a list file(.yaml, .yml, .xml) for image pairs.\n";
        cin >> input;
    }

    fsys::path path1(input);
    if (isFileType(path1, {".yml", ".yaml", ".xml"})) {
        if (!loadImgPair_YML(path1, lImgs, rImgs)) {
            cout << "Fail to load image pairs from this file. "
                "Please check the correctness of the file path and the file content.\n";
            return false;
        }
        if (lImgs.size() != rImgs.size()) {
            cout << "Fail to align images. lImgs.size()!=rImgs.size().\n";
            return false;
        }
        return true;
    }
    else {
        if (!getImgList(path1, lImgs)) {
            cout << "Fail to load left images list. Please check the correctness of the path.\n";
            return false;
        }
    }

    input = parser.get<string>(1);
    if (input == string()) {
        cout << "Please enter the directory or the path of right image.\n";
        cin >> input;
    }
    fsys::path path2(input);
    if(!getImgList(path2, rImgs)) {
        cout << "Fail to load right images list. Please check the correctness of the path.\n";
        return false;
    }
    if (lImgs.size() != rImgs.size()) {
        cout << "Fail to align images. lImgs.size()!=rImgs.size().\n";
        return false;
    }
    return true;
}

CommandLineEngine::CommandLineEngine(CommandLineParser &_parser)
    : parser(_parser)
{
    alg = createAlg(parser.get<string>("algorithm"));
    if(!getImageList(lImgs, rImgs)) {
        cout << "Initialization interrupted. Failed to retrieve the image list.";
        std::terminate();
    }
}

CommandLineEngine::~CommandLineEngine() {
    delete alg;
}

int CommandLineEngine::run() {
    size_t idx = 0;
    size_t idxMax = lImgs.size();
    while(1) {
        step Direction = process(lImgs[idx], rImgs[idx]);
        if (Direction == Exit)
            break;
        switch (Direction)
        {
        case GoNext:
            idx++;
            if (idx >= idxMax)
                idx = 0;
            break;
        case GoBack:
            idx--;
            if (idx < 0)
                idx = idxMax-1;
            break;
        default:
            break;
        }
    }
    return 0;
}

AlgConsoleInterface *CommandLineEngine::createAlg(string type) {
    if (type == "SGBM") {
        return new SGBM_Console();
    }
    else if (type == "BM") {
        return new BM_Console();
    }
    else if (type == "ELAS") {
        return new ELAS_Console();
    }
    else {
        return nullptr;
    }
}

void CommandLineEngine::changeAlg(string type) {
    if (!alg) {
        alg = createAlg(type);
        return;
    }
    if (alg->getType() == type) {
        return;
    }
    delete alg;
    alg = createAlg(type);
}

SGBM_Console::SGBM_Console() {
    algorithm_ptr = new Alg_SGBM_Impl();
}

void SGBM_Console::showSetting() {
    Alg_SGBM_Impl *sgbm = (Alg_SGBM_Impl*) algorithm_ptr;
    printf("Algorithm: SGBM\n");
    printf("1 %s: %d\n", "Block Size", sgbm->alg->getBlockSize());
    printf("2 %s: %d\n", "Disparities numbers", sgbm->alg->getNumDisparities());
    printf("3 %s: %d\n", "Minimum Disparity", sgbm->alg->getMinDisparity());
    printf("4 %s: %d\n", "P1", sgbm->alg->getP1());
    printf("5 %s: %d\n", "P2", sgbm->alg->getP2());
    printf("6 %s: %d\n", "PreFilterCap", sgbm->alg->getPreFilterCap());
    printf("7 %s: %d\n", "Uniqueness Ratio", sgbm->alg->getUniquenessRatio());
    printf("8 %s: %d\n", "Speckle Window Size", sgbm->alg->getSpeckleWindowSize());
    printf("9 %s: %d\n", "Speckle Range", sgbm->alg->getSpeckleRange());
    printf("10 %s: %d\n", "disp12MaxDiff", sgbm->alg->getDisp12MaxDiff());
    printf("11 mode: ");
    switch (sgbm->alg->getMode()) {
    case StereoSGBM::MODE_SGBM:
        cout << "MODE_SGBM";
        break;
    case StereoSGBM::MODE_HH:
        cout << "MODE_HH";
        break;
    case StereoSGBM::MODE_SGBM_3WAY:
        cout << "MODE_SGBM_3WAY";
        break;
    case StereoSGBM::MODE_HH4:
        cout << "MODE_HH4";
        break;
    default:
        break;
    }
    cout << endl;
}

void SGBM_Console::modifySetting() {
    Alg_SGBM_Impl *sgbm = (Alg_SGBM_Impl*) algorithm_ptr;
    while(1) {
        system("cls");
        showSetting();
        printf("0 run\n");
        cout << "Please enter the attribute index to be modified:\n";
        int idx;
        cin >> idx;
        if (idx == 0)
            return;
        int value;
        if (idx > 0 && idx < 11) {
            cout << "Please enter the value:\n";
            cin >> value;
        }
        switch (idx) {
        case 1:
            sgbm->alg->setBlockSize(value);
            break;
        case 2:
            sgbm->alg->setNumDisparities(value);
            break;
        case 3:
            sgbm->alg->setMinDisparity(value);
            break;
        case 4:
            sgbm->alg->setP1(value);
            break;
        case 5:
            sgbm->alg->setP2(value);
            break;
        case 6:
            sgbm->alg->setPreFilterCap(value);
            break;
        case 7:
            sgbm->alg->setUniquenessRatio(value);
            break;
        case 8:
            sgbm->alg->setSpeckleWindowSize(value);
            break;
        case 9:
            sgbm->alg->setSpeckleRange(value);
            break;
        case 10:
            sgbm->alg->setDisp12MaxDiff(value);
            break;
        case 11:
        {
            cout << "Please select the mode:\n"
            "1 MODE_SGBM\n"
            "2 MODE_HH\n"
            "3 MODE_SGBM_3WAY\n"
            "4 MODE_HH4\n";
            int mode;
            cin >> mode;
            mode--;
            sgbm->alg->setMode(mode);
            break;
        }
        default:
            break;
        }
    }
}

BM_Console::BM_Console() {
    algorithm_ptr = new Alg_BM_Impl();
}

void BM_Console::showSetting() {}

void BM_Console::modifySetting() {}

ELAS_Console::ELAS_Console() {}

void ELAS_Console::showSetting() {}

void ELAS_Console::modifySetting() {}