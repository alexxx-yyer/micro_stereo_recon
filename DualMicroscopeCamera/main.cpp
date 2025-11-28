#include <iostream>

//for getting the information of cameras
#include <dshow.h>
#include <comutil.h>

//for managing the file directory
#include <ctime>
#include <filesystem>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"

#include "analyse.h"

using namespace std;
using namespace cv;

const string keys =
"{help h||print the help message}"
"{@count c|0|the begin number for record image}"
"{@sroot s||the directory for save images}"
"{@file_name n|sample|the file name used in saved images}"
"{fc||find chessboard corners}"
"{fg||find grid}"
"{b||capture background}";

const string about =
"\nThis program is developed for recording images which would be use in camera calibrating task.\n"
"\nYou can change the saving path through command line\n"
" -s=\"path-to-save\"\n"
"\nAnd you can change the begin number of image use\n"
" -c=\"number\"\n"
"\nAnd you can modify the file name of images that are going to save\n"
" -n=\"file name\"\n"
"\nThe default value of save root and image begin number as follows\n";

void check_Connected_Camera(int* idx_l, int* idx_r);
void getParams(VideoCapture* camera);
void modifyPropUp(VideoCapture* camera1, VideoCapture* camera2, int prop);
void modifyPropDown(VideoCapture* camera1, VideoCapture* camera2, int prop);

int main(int argc, char* argv[])
{
    CommandLineParser parser(argc, argv, keys);
    if (parser.has("help"))
    {
        parser.about(about);
        parser.printMessage();
        return 0;
    }

    //initialize the save path
    string sroot = parser.get<string>("@sroot");
    if (sroot == "\0") {
        std::cout << "Please enter the root directory for storing captured images:\n";
        cin >> sroot;
    }
    bool background = parser.get<bool>("b");
    bool FC = parser.get<bool>("fc");
    bool FG = parser.get<bool>("fg");
    Size boardsize;
    if (FC) {
        cout << "Please enter the width of inner corners:\n";
        cin >> boardsize.width;
        cout << "Please enter the height of inner corners:\n";
        cin >> boardsize.height;
    }
    filesystem::path file_directory(sroot);
    int count = parser.get<int>("@count");
    string file_name = parser.get<string>("@file_name");
    if (background) {
        file_name = "BG";
    }
    filesystem::path file_path;

    //automatically select the camera
    int* left_idx = new int();
    int* right_idx = new int();
    check_Connected_Camera(left_idx, right_idx);

    VideoCapture camera_l(*left_idx);
    VideoCapture camera_r(*right_idx);
    if (!camera_l.isOpened()) {
        cout << "left camera cannot open" << endl;
        return 0;
    }
    if (!camera_r.isOpened()) {
        cout << "right camera cannot open" << endl;
        return 0;
    }
    camera_l.set(CAP_PROP_FRAME_WIDTH, 1920);
    camera_l.set(CAP_PROP_FRAME_HEIGHT, 1440);
    camera_r.set(CAP_PROP_FRAME_WIDTH, 1920);
    camera_r.set(CAP_PROP_FRAME_HEIGHT, 1440);
    double l_exposure = camera_l.get(CAP_PROP_EXPOSURE);
    double r_exposure = camera_r.get(CAP_PROP_EXPOSURE);
    double l_brightness = camera_l.get(CAP_PROP_BRIGHTNESS);
    double r_brightness = camera_r.get(CAP_PROP_BRIGHTNESS);
    int prop = CAP_PROP_EXPOSURE;
    Mat img_l, img_r, canva;
    namedWindow("camera", WINDOW_NORMAL);
    char number[5] = { 0 };
    while (1) {
        camera_l >> img_l;
        camera_r >> img_r;
        canva = Mat(Size(img_l.cols + img_r.cols, img_l.rows), CV_8UC3);
        img_l.copyTo(canva(Rect(0, 0, img_l.cols, img_l.rows)));
        img_r.copyTo(canva(Rect(img_l.cols, 0, img_r.cols, img_r.rows)));
        imshow("camera", canva);
        int key = waitKey(10);
        if (key == 'q')
            break;
        else if (key == 's') {
            filesystem::path lpath = file_directory / filesystem::path("left");
            filesystem::path rpath = file_directory / filesystem::path("right");
            if (background) {
                lpath /= filesystem::path("Background");
                rpath /= filesystem::path("Background");
            }
            if (!filesystem::exists(lpath) || !filesystem::exists(rpath)) {
                filesystem::create_directories(lpath);
                filesystem::create_directories(rpath);
            }
            sprintf_s<5>(number, "_%03d", count);
            file_path = lpath / filesystem::path(file_name + (string)number + "_left.png");
            imwrite(file_path.string(), img_l);
            cout << "left image has been saved as " << file_path << endl;
            file_path = rpath / filesystem::path(file_name + (string)number + "_right.png");
            imwrite(file_path.string(), img_r);
            cout << "right image has been saved as " << file_path << endl;
            count++;
        }
        else if (key == 'f') {
            if (FC) {
                vector<Point2f> corners1, corners2;
                //bool found1 = findChessboardCorners(img_l, boardsize, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
                //bool found2 = findChessboardCorners(img_r, boardsize, corners2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
                bool found1 = findChessboardCornersSB(img_l, boardsize, corners1);
                bool found2 = findChessboardCornersSB(img_r, boardsize, corners2);
                if (found1)
                {
                    drawChessboardCorners(img_l, boardsize, Mat(corners1), found1);
                    namedWindow("corners1", WINDOW_NORMAL);
                    imshow("corners1", img_l);
                }
                if (found2)
                {
                    drawChessboardCorners(img_r, boardsize, Mat(corners2), found2);
                    namedWindow("corners2", WINDOW_NORMAL);
                    imshow("corners2", img_r);
                }
            }
            else if (FG) {
                Mat gray;
                cvtColor(img_l, gray, COLOR_BGR2GRAY);
                Mat res = segment(gray);
                namedWindow("seg1", WINDOW_NORMAL);
                imshow("seg1", res);
                cvtColor(img_r, gray, COLOR_BGR2GRAY);
                res = segment(gray);
                namedWindow("seg2", WINDOW_NORMAL);
                imshow("seg2", res);
            }
        }
        else if (key == 'p') {
            printf("Camera L:\n");
            getParams(&camera_l);
            printf("\nCamera R:\n");
            getParams(&camera_r);
        }
        else if (key == 'c') {
            printf(
                "Select one following properties to modify it:\n"
                "%d CAP_PROP_EXPOSURE\n"
                "%d CAP_PROP_BRIGHTNESS\n",
                CAP_PROP_EXPOSURE,
                CAP_PROP_BRIGHTNESS
            );
            cin >> prop;
        }
        else if (key == ']') {
            modifyPropUp(&camera_l, &camera_r, prop);
        }
        else if (key == '[') {
            modifyPropDown(&camera_l, &camera_r, prop);
        }
    }
    delete left_idx, right_idx;

    return 0;
}

void check_Connected_Camera(int* idx_l, int* idx_r) {
    string Lcamera_name = "ARM MicroscopeL";
    string Rcamera_name = "ARM MicroscopeR";
    HRESULT hr = CoInitialize(NULL);
    if (FAILED(hr)) {
        printf("failed in 1");
        return;
    }
    
    ICreateDevEnum* pDevEnum;
    hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pDevEnum));
    if (FAILED(hr)) {
        printf("failed in 2");
        return;
    }

    IEnumMoniker* pEnum;
    hr = pDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnum, 0);
    if (hr == S_FALSE) {
        printf("failed in 3");
        return;
    }

    IMoniker* pMoniker = NULL;
    int count = 0;
    while (pEnum->Next(1, &pMoniker, NULL) == S_OK) {
        IPropertyBag* pPropBag;
        hr = pMoniker->BindToStorage(0, 0, IID_PPV_ARGS(&pPropBag));
        if (FAILED(hr)) {
            printf("failed in 4");
            pMoniker->Release();
            continue;
        }

        cout << endl << "device Index: " << count << endl;
        VARIANT var;
        VariantInit(&var);

        hr = pPropBag->Read(L"FriendlyName", &var, 0);
        if (SUCCEEDED(hr)) {
            char* ptr = _com_util::ConvertBSTRToString(var.bstrVal);
            if ((string)ptr == (string)Lcamera_name)
                *idx_l = count;
            if ((string)ptr == (string)Rcamera_name)
                *idx_r = count;
            cout << "FriendlyName: " << ptr << endl;
            VariantClear(&var);
        }
        hr = pPropBag->Read(L"DevicePath", &var, 0);
        if (SUCCEEDED(hr)) {
            char* ptr = _com_util::ConvertBSTRToString(var.bstrVal);
            cout << "DevicePath: " << ptr << endl;
            VariantClear(&var);
        }
        count++;
        cout << endl;

        pPropBag->Release();
        pMoniker->Release();
    }
    cout << "left camera idx: " << *idx_l << endl << "right camera idx: " << *idx_r << endl << endl;

    pEnum->Release();
    pDevEnum->Release();
    CoUninitialize();
    return;
}

void getParams(VideoCapture* camera) {
    printf(
        "Exposure: %f\n"
        "Brightness: %f\n"
        "Focus: %f\n",
        camera->get(CAP_PROP_EXPOSURE),
        camera->get(CAP_PROP_BRIGHTNESS),
        camera->get(CAP_PROP_FOCUS)
    );
}

void modifyPropUp(VideoCapture* camera1, VideoCapture* camera2, int prop) {
    switch (prop)
    {
    case CAP_PROP_EXPOSURE:
    {
        //camera1->set(CAP_PROP_EXPOSURE, camera1->get(CAP_PROP_EXPOSURE) + 1.);
        //camera2->set(CAP_PROP_EXPOSURE, camera2->get(CAP_PROP_EXPOSURE) + 1.);
        camera1->set(CAP_PROP_EXPOSURE, camera1->get(CAP_PROP_EXPOSURE) + 1.);
        camera2->set(CAP_PROP_EXPOSURE, camera2->get(CAP_PROP_EXPOSURE) + 1.);
        break;
    }
    case CAP_PROP_BRIGHTNESS:
    {
        camera1->set(CAP_PROP_BRIGHTNESS, camera1->get(CAP_PROP_BRIGHTNESS) + 1.);
        camera2->set(CAP_PROP_BRIGHTNESS, camera2->get(CAP_PROP_BRIGHTNESS) + 1.);
        break;
    }
    default:
        break;
    }
}

void modifyPropDown(VideoCapture* camera1, VideoCapture* camera2, int prop) {
    switch (prop)
    {
    case CAP_PROP_EXPOSURE:
    {
        camera1->set(CAP_PROP_EXPOSURE, camera1->get(CAP_PROP_EXPOSURE) - 1.);
        camera2->set(CAP_PROP_EXPOSURE, camera2->get(CAP_PROP_EXPOSURE) - 1.);
        break;
    }
    case CAP_PROP_BRIGHTNESS:
    {
        camera1->set(CAP_PROP_BRIGHTNESS, camera1->get(CAP_PROP_BRIGHTNESS) - 1.);
        camera2->set(CAP_PROP_BRIGHTNESS, camera2->get(CAP_PROP_BRIGHTNESS) - 1.);
        break;
    }
    default:
        break;
    }
}