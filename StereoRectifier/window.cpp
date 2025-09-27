#include "window.h"
#include "ui_window.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QtCore5Compat/QTextCodec>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"

#include "StereoMatchingTools.h"
#include "VisualizationTools.h"
#include "DirTools.h"

#include <iostream>
#include <filesystem>

using namespace std;
using namespace cv;

// 将QString转换为std::string
string QStr_to_string(QString qstr) {
    QTextCodec* codec = QTextCodec::codecForName("GBK");
    if (!codec) {
        throw std::runtime_error("GBK codec not found");
    }

    QByteArray gbkByte = codec->fromUnicode(qstr);
    return string(gbkByte.constData(), gbkByte.size());
}

// 构造函数，初始化窗口
window::window(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::window)
    , IFlag(INTER_CUBIC)
{
    ui->setupUi(this);
    ui->imgWidth->setText(QString::number(0));
    ui->imgHeight->setText(QString::number(0));
    ui->alpha->setDisabled(true);
    ui->manualSave->setDisabled(true);
}

// 析构函数，释放UI资源
window::~window()
{
    delete ui;
}

// 加载参数文件
void window::on_loadParams_clicked(bool checked) {
    QString path = QFileDialog::getOpenFileName(this, "Load Parameters", "", "*.yml *.xml *.yaml");
    if (path.isEmpty()) {
        return;
    }
    FileStorage fs(QStr_to_string(path), FileStorage::READ);
    CheckAndLoad(fs, "M1", M1);
    CheckAndLoad(fs, "D1", D1);
    CheckAndLoad(fs, "M2", M2);
    CheckAndLoad(fs, "D2", D2);
    CheckAndLoad(fs, "R", R);
    CheckAndLoad(fs, "T", T);
    CheckAndLoad(fs, "imgsize", oldsize);
    fs.release();
    ui->lineEdit->setText(path);
    newsize = oldsize;
    updateSizeGui();
    ui->presetSizes->setCurrentIndex(0);
}

// 加载图像对
void window::on_loadImages_clicked(bool checked) {
    lImgs.clear();
    rImgs.clear();
    QString path = QFileDialog::getOpenFileName(this, "Load Images", "", "*.yml *.xml *.yaml");
    if (path.isEmpty()) {
        return;
    }
    if (!loadImgPair_YML(QStr_to_string(path), lImgs, rImgs)) {
        QMessageBox::information(this, "Caution", "Load images failed");
        return;
    }
    printf("%d pair images loaded.\n", static_cast<int>(lImgs.size()));
    ui->lineEdit_2->setText(path);
}

// 选择输出目录
void window::on_outputDir_clicked(bool checked) {
    QString path = QFileDialog::getExistingDirectory(this, "Choose Output Directory", ui->lineEdit->text());
    ui->lineEdit_3->setText(path);
}

// 立体校正
void window::on_rectify_clicked(bool checked) {
    if (newsize.empty()) {
        QMessageBox::information(this, "Caution", "The target image size is necessary");
        return;
    }
    stereoRectify(M1, D1, M2, D2, newsize, R, T, R1, R2, P1, P2, Q,
        ui->calibZeroDisp->isChecked() ? CALIB_ZERO_DISPARITY : 0,
        ui->checkAlpha->isChecked() ? ui->alpha->value() : -1,
        newsize, &roi1, &roi2);

    Mat omap1, omap2, nmap1, nmap2;
    initUndistortRectifyMap(M1, D1, R1, P1, newsize, CV_32FC1, omap1, nmap1);
    initUndistortRectifyMap(M2, D2, R2, P2, newsize, CV_32FC1, omap2, nmap2);

    QString dir = ui->lineEdit_3->text();
    for (size_t i = 0; i < lImgs.size(); i++) {
        Mat imgL = imread(lImgs[i]);
        Mat imgR = imread(rImgs[i]);

        Mat recL, recR;
        remap(imgL, recL, omap1, nmap1, IFlag);
        remap(imgR, recR, omap2, nmap2, IFlag);

        string nameL = "rec_" + filesystem::path(lImgs[i]).filename().string();
        string nameR = "rec_" + filesystem::path(rImgs[i]).filename().string();

        if (ui->showRectified->isChecked()) {
            showRectified(recL, recR, 10, "Rectified");
            if (ui->manualSave->isChecked()) {
                int key = waitKey();
                if (key == 'q') {
                    destroyWindow("Rectified");
                    break;
                }
                else if (key != 's')
                    continue;
            }
            else {
                waitKey(100);
            }
        }

        if (dir.isEmpty()) {
            QMessageBox::information(this, "Caution", "Output directory is necessary!");
            return;
        }
        string pathL = QStr_to_string(dir) + "/" + nameL;
        string pathR = QStr_to_string(dir) + "/" + nameR;
        printf("save: %s\n", pathL.c_str());
        printf("save: %s\n", pathR.c_str());
        imwrite(pathL, recL);
        imwrite(pathR, recR);
    }
}

// 更新图像宽度
void window::on_imgWidth_editingFinished() {
    newsize.width = ui->imgWidth->text().toInt();
    ui->presetSizes->setCurrentIndex(1);
}

// 更新图像高度
void window::on_imgHeight_editingFinished() {
    newsize.height = ui->imgHeight->text().toInt();
    ui->presetSizes->setCurrentIndex(1);
}

// 显示校正图像切换
void window::on_showRectified_toggled(bool checked) {
    ui->manualSave->setDisabled(!checked);
}

// Alpha值切换
void window::on_checkAlpha_toggled(bool checked) {
    ui->alpha->setDisabled(!checked);
}

// 预设尺寸切换
void window::on_presetSizes_currentIndexChanged(int index) {
    switch (index)
    {
    case 0:
        newsize = oldsize;
        updateSizeGui();
        break;
    case 1:
        break;
    default:
        break;
    }
}

// 保存参数文件
void window::on_saveParams_clicked(bool checked) {
    QString recommend = ui->lineEdit->text();
    QString basename = "rectifiedParams.yml";
    for (qsizetype i = recommend.size() - 1; i > 0; i--) {
        if (recommend[i] == "/" || recommend[i] == "\\") {
            recommend.truncate(i + 1);
            break;
        }
    }
    QString path = QFileDialog::getSaveFileName(this, "Save Parameters", recommend + basename);
    FileStorage fs(QStr_to_string(path), FileStorage::WRITE);
    fs << "R1" << R1
        << "P1" << P1
        << "R2" << R2
        << "P2" << P2
        << "Q" << Q
        << "roi1" << roi1
        << "roi2" << roi2;
    fs.release();
}

// 更新GUI中的图像尺寸
inline void window::updateSizeGui() {
    ui->imgWidth->setText(QString::number(newsize.width));
    ui->imgHeight->setText(QString::number(newsize.height));
}