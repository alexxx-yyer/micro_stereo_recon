#include "panel.h"
#include "ui_panel.h"
#include <iostream>
#include <bitset>
#include "DirTools.h"
#include "StereoMatchingTools.h"

#include <QFileDialog>
#include <QtCore5Compat/QTextCodec>
#include <QMessageBox>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

static void drawCorners(Mat& canva, vector<Point2f>& corners) {
    CV_Assert(canva.type() == CV_8UC3);
    circle(canva, corners[0], 5, Scalar(0, 255, 255));
    for (int i = 1; i < corners.size(); i++) {
        circle(canva, corners[i], 5, Scalar(0, 255, 0));
    }
}

string QStr_to_string(QString qstr) {
    QTextCodec* codec = QTextCodec::codecForName("GBK");
    if (!codec) {
        throw std::runtime_error("GBK codec not found");
    }

    QByteArray gbkByte = codec->fromUnicode(qstr);
    return string(gbkByte.constData(), gbkByte.size());
}

inline void panel::modifyFlag(int sign, bool checked) {
    if (checked) {
        flag = flag | sign;
    }
    else {
        flag = flag & ~sign;
    }
#ifdef _DEBUG
    bitset<32> bits(flag);
    cout << bits << endl;
#endif
}

double panel::getPrecision() {
    int idx = ui->precision->currentIndex();
    return pow(10, -(idx+1));
}


panel::panel(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::panel)
    , flag(CALIB_USE_INTRINSIC_GUESS +
        CALIB_FIX_ASPECT_RATIO +
        CALIB_SAME_FOCAL_LENGTH +
        CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5 +
        CALIB_RATIONAL_MODEL +
        CALIB_ZERO_TANGENT_DIST)
    , criteria(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 1000, 1e-6))
{
    ui->setupUi(this);
    connect(ui->setCols, &QSpinBox::editingFinished, this, &panel::init_objPoints);
    connect(ui->setRows, &QSpinBox::editingFinished, this, &panel::init_objPoints);
    connect(ui->setInterval, &QDoubleSpinBox::editingFinished, this, &panel::init_objPoints);
}

panel::~panel()
{
    delete ui;
}

void panel::on_loadFile_clicked(bool checked) {
    lImgs.clear();
    rImgs.clear();
    QString filepath = QFileDialog::getOpenFileName(this, QString(), QString(), { "*.yml" });
    loadImgPair_YML(QStr_to_string(filepath), lImgs, rImgs);
    ui->lineEdit->setText(filepath);
    printf("load from file: %s\n", QStr_to_string(filepath).c_str());
    printf("%d left images.\n", static_cast<int>(lImgs.size()));
    printf("%d right images.\n", static_cast<int>(rImgs.size()));
    ui->imgNums->setText(QString::number(lImgs.size()));
}

void panel::init_objPoints() {
    int w = ui->setCols->value();
    int h = ui->setRows->value();
    float interval = ui->setInterval->value();
    initPatternCoordinates(w, h, interval, objPoints);
    qDebug("%d points generated.", objPoints.size());
#ifdef _DEBUG
    for (int i = 0; i < objPoints.size(); i++) {
        cout << objPoints[i];
    }
#endif
}

void panel::on_detectCorners_clicked(bool checked) {
    corners1.clear();
    corners2.clear();
    if (lImgs.empty() || rImgs.empty()) {
        QMessageBox::information(this, "Caution", "Please load images first!");
        return;
    }
    if (lImgs.size() != rImgs.size()) {
        QMessageBox::information(this, "Caution", "Images number have not aligned, please check the image list.");
        return;
    }
    int w = ui->setCols->value();
    int h = ui->setRows->value();
    int count = 0;
    for (int i = 0; i < lImgs.size(); i++) {
        Mat lImg = imread(lImgs[i], ui->toGray->isChecked() ? IMREAD_GRAYSCALE : IMREAD_ANYCOLOR);
        Mat rImg = imread(rImgs[i], ui->toGray->isChecked() ? IMREAD_GRAYSCALE : IMREAD_ANYCOLOR);
        if (imgsize != lImg.size()) {
            imgsize = lImg.size();
        }
        vector<Point2f> lcorners, rcorners;
        if (!findChessboardCornersSB(lImg, Size(w, h), lcorners) ||
            !findChessboardCornersSB(rImg, Size(w, h), rcorners))
        {
            printf("Fail   :%s\n", lImgs[i].c_str());
            printf("Fail   :%s\n", rImgs[i].c_str());
            badIdx.push_back(i);
            continue;
        }
        Point2f vec1 = lcorners[1] - lcorners[0];
        Point2f vec2 = rcorners[1] - rcorners[0];
        if (vec1.dot(vec2) < 0) {
            std::reverse(rcorners.begin(), rcorners.end());
        }
        printf("Success:%s\n", lImgs[i].c_str());
        printf("Success:%s\n", rImgs[i].c_str());
        corners1.push_back(lcorners);
        corners2.push_back(rcorners);
        count += w * h;
    }
    ui->cornerNums->setText(QString::number(count));
    vector<vector<Point3f>> _objPoints;
    _objPoints.resize(corners1.size(), objPoints);
    M1 = initCameraMatrix2D(_objPoints, corners1, imgsize, 0);
    M2 = initCameraMatrix2D(_objPoints, corners2, imgsize, 0);
}

void panel::on_iterNum_editingFinished() {
    criteria.maxCount = ui->iterNum->value();
#ifdef _DEBUG
    printf("Terminate Criteria: max iteration %d, eps %.8f\n", criteria.maxCount, criteria.epsilon);
#endif
}

void panel::on_eps_editingFinished() {
    criteria.epsilon = ui->eps->value() * getPrecision();
#ifdef _DEBUG
    printf("Terminate Criteria: max iteration %d, eps %.8f\n", criteria.maxCount, criteria.epsilon);
#endif
}

void panel::on_precision_currentIndexChanged(int index) {
    criteria.epsilon = ui->eps->value() * getPrecision();
#ifdef _DEBUG
    printf("Terminate Criteria: max iteration %d, eps %.8f\n", criteria.maxCount, criteria.epsilon);
#endif
}


void panel::on_nintrinsics_toggled(bool checked) {
    modifyFlag(CALIB_NINTRINSIC, checked);
}

void panel::on_intrinsicsGuess_toggled(bool checked) {
    modifyFlag(CALIB_USE_INTRINSIC_GUESS, checked);
}

void panel::on_fixAspectRatio_toggled(bool checked) {
    modifyFlag(CALIB_FIX_ASPECT_RATIO, checked);
}

void panel::on_fixPrincipalPoint_toggled(bool checked) {
    modifyFlag(CALIB_FIX_PRINCIPAL_POINT, checked);
}

void panel::on_fixFocalLength_toggled(bool checked) {
    modifyFlag(CALIB_FIX_FOCAL_LENGTH, checked);
}

void panel::on_sameFocalLength_toggled(bool checked) {
    modifyFlag(CALIB_SAME_FOCAL_LENGTH, checked);
}

void panel::on_fixK1_toggled(bool checked) {
    modifyFlag(CALIB_FIX_K1, checked);
}

void panel::on_fixK2_toggled(bool checked) {
    modifyFlag(CALIB_FIX_K2, checked);
}

void panel::on_fixK3_toggled(bool checked) {
    modifyFlag(CALIB_FIX_K3, checked);
}

void panel::on_fixK4_toggled(bool checked) {
    modifyFlag(CALIB_FIX_K4, checked);
}

void panel::on_fixK5_toggled(bool checked) {
    modifyFlag(CALIB_FIX_K5, checked);
}

void panel::on_fixK6_toggled(bool checked) {
    modifyFlag(CALIB_FIX_K6, checked);
}

void panel::on_fixS1S2S3S4_toggled(bool checked) {
    modifyFlag(CALIB_FIX_S1_S2_S3_S4, checked);
}

void panel::on_rationalModel_toggled(bool checked) {
    modifyFlag(CALIB_RATIONAL_MODEL, checked);
}

void panel::on_thinPrismModel_toggled(bool checked) {
    modifyFlag(CALIB_THIN_PRISM_MODEL, checked);
}

void panel::on_tiltedModel_toggled(bool checked) {
    modifyFlag(CALIB_TILTED_MODEL, checked);
}

void panel::on_fixTAUXTAUY_toggled(bool checked) {
    modifyFlag(CALIB_FIX_TAUX_TAUY, checked);
}

void panel::on_useQR_toggled(bool checked) {
    modifyFlag(CALIB_USE_QR, checked);
}

void panel::on_useLU_toggled(bool checked) {
    modifyFlag(CALIB_USE_LU, checked);
}

void panel::on_fixTangentDist_toggled(bool checked) {
    modifyFlag(CALIB_FIX_TANGENT_DIST, checked);
}

void panel::on_zeroTangentDist_toggled(bool checked) {
    modifyFlag(CALIB_ZERO_TANGENT_DIST, checked);
}

void panel::on_fixIntrinsics_toggled(bool checked) {
    modifyFlag(CALIB_FIX_INTRINSIC, checked);
}

void panel::on_zeroDisparity_toggled(bool checked) {
    modifyFlag(CALIB_ZERO_DISPARITY, checked);
}

void panel::on_extrinsicsGuess_toggled(bool checked) {
    modifyFlag(CALIB_USE_EXTRINSIC_GUESS, checked);
}

void panel::on_calculate_clicked(bool checked) {
    if (corners1.empty() || corners2.empty()) {
        QMessageBox::information(this, "Caution", "It is necessary to perform corner detection before calculating the camera parameters!");
    }
    vector<vector<Point3f>> _objPoints;
    _objPoints.resize(corners1.size(), objPoints);

    cout << "calculating...\n";
    double rms = stereoCalibrate(_objPoints, corners1, corners2, M1, D1, M2, D2, imgsize, R, T, E, F, perError, flag, criteria);
    cout << perError.type() << endl;
    printf("Errors:\n"
        "%7s     %7s\n", "left", "right");
    for (int i = 0; i < perError.rows; i++) {
        double* err = perError.ptr<double>(i);
        printf("%7f  %7f\n", err[0], err[1]);
    }
    printf("Average Error: %7f\n", rms);
}

void panel::on_resetParams_clicked(bool checked) {
    vector<vector<Point3f>> _objPoints;
    _objPoints.resize(corners1.size(), objPoints);
    M1 = initCameraMatrix2D(_objPoints, corners1, imgsize, 0);
    M2 = initCameraMatrix2D(_objPoints, corners2, imgsize, 0);
}


void panel::on_save_clicked(bool checked) {
    QString recommend = ui->lineEdit->text();
    QString basename = "stereoParams.yml";
    for (qsizetype i = recommend.size()-1; i > 0; i--) {
        if (recommend[i] == "/" || recommend[i] == "\\") {
            recommend.truncate(i + 1);
            break;
        }
    }
    QString path = QFileDialog::getSaveFileName(this, "Save File", recommend + basename);
    FileStorage fs(QStr_to_string(path), FileStorage::WRITE);
    fs << "M1" << M1
        << "D1" << D1
        << "M2" << M2
        << "D2" << D2
        << "R" << R
        << "T" << T
        << "E" << E
        << "F" << F
        << "imgsize" << imgsize;
    fs.release();
}

void panel::on_saveCorners_clicked(bool checked) {
    QString recommend = ui->lineEdit->text();
    for (qsizetype i = recommend.size()-1; i > 0; i--) {
        if (recommend[i] == "/" || recommend[i] == "\\") {
            recommend.truncate(i + 1);
            break;
        }
    }
    QString dir = QFileDialog::getExistingDirectory(this, "Save", recommend);
    vector<int>::const_iterator skipIdx = badIdx.cbegin();
    vector<vector<Point2f>>::iterator l_iter = corners1.begin();
    vector<vector<Point2f>>::iterator r_iter = corners2.begin();
    for (int i = 0; i < lImgs.size(); i++) {
        if (i == *skipIdx) {
            skipIdx++;
            continue;
        }
        Size pt_size(ui->setCols->value(), ui->setRows->value());
        Mat img = imread(lImgs[i]);
        drawCorners(img, *l_iter);
        filesystem::path file(lImgs[i]);
        string spath = QStr_to_string(dir) + "/" + file.filename().string();
        cout << "save: " << spath << endl;
        imwrite(spath, img);
        l_iter++;

        img = imread(rImgs[i]);
        drawCorners(img, *r_iter);
        file = filesystem::path(rImgs[i]);
        spath = QStr_to_string(dir) + "/" + file.filename().string();
        cout << "save: " << spath << endl;
        imwrite(spath, img);
        r_iter++;
    }
}
