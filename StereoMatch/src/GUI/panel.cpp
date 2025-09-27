#include "panel.h"
#include "fileIO.h"
#include "LoadWidget.h"
#include "AlgWidget.h"
#include "ui_panel.h"
#include <QFileDialog>
#include <QTextCodec>
#include <QMessageBox>
#include "DirTools.h"
#include "VisualizationTools.h"
#include "StereoMatchingTools.h"
#include "OCV_PCL.h"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;
using namespace pcl;
namespace fsys = filesystem;

string QStr_to_string(QString qstr) {
    QTextCodec *codec = QTextCodec::codecForName("GBK");
    if (!codec) {
        throw std::runtime_error("GBK codec not found");
    }

    QByteArray gbkByte = codec->fromUnicode(qstr);
    return string(gbkByte.constData(), gbkByte.size());
}

panel::panel(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::panel)
    , idx(0)
    , disp(new Mat)
    , disp8(new Mat)
    , imgL(new Mat)
    , imgR(new Mat)
    , cloud(new PointCloud<PointXYZRGB>)
{
    ui->setupUi(this);
    alg = new sgbm_setting(ui->settingGroup);
    alg->updateSetting();
    createLoadFile();
    connect(ui->listMode, &QRadioButton::toggled, this, &panel::on_loadlist);
    connect(ui->dirMode, &QRadioButton::toggled, this, &panel::on_loadfile);
    connect(ui->QFile, &QPushButton::clicked, this, &panel::loadQMatrix);

    connect(ui->runButton, &QPushButton::clicked, this, &panel::match);
    connect(ui->previousButton, &QPushButton::clicked, this, &panel::on_previous_clicked);
    connect(ui->nextButton, &QPushButton::clicked, this, &panel::on_next_clicked);
    connect(ui->cloudButton, &QPushButton::clicked, this, &panel::on_cloud_clicked);

    connect(ui->saveButton, &QPushButton::clicked, this, &panel::on_save_clicked);
    connect(ui->savePathButton, &QPushButton::clicked, this, &panel::selectDispSavePath);
    connect(ui->cloudPathButton, &QPushButton::clicked, this, &panel::selectCloudSavePath);
    connect(ui->isSaveCloud, QOverload<int>::of(&QCheckBox::stateChanged), this, &panel::on_saveCloud_checked);

    connect(ui->SGBMButton, &QRadioButton::toggled, this, &panel::on_sgbm);
    connect(ui->BMButton, &QRadioButton::toggled, this, &panel::on_bm);
}

panel::~panel()
{
    delete ui;
    delete loader;
    delete alg;
    delete disp;
    delete disp8;
    delete imgL;
    delete imgR;
}

void panel::updateTotal() {
    string msg = "/" + to_string(lImgs.size());
    ui->total->setText(QString::fromUtf8(msg.c_str()));
}

void panel::on_loadlist(bool checked) {
    if (!checked)
        return;
    delete loader;
    createLoadList();
    loader->show();
}

void panel::on_loadfile(bool checked) {
    if (!checked)
        return;
    delete loader;
    loader = new LoadFile(ui->loaderGroup);
    loader->show();
}

void panel::loadlistfile(QString path) {
    if (path.isEmpty())
        return;
    lImgs.clear();
    rImgs.clear();
    string file = QStr_to_string(path);
    loadImgPair_YML(file, lImgs, rImgs);
    updateTotal();
    *imgL = imread(lImgs[0]);
    *imgR = imread(rImgs[0]);
}

void panel::loadleftfile(QString path) {
    if (path.isEmpty())
        return;
    lImgs.clear();
    string file = QStr_to_string(path);
    getImgList(file, lImgs);
    updateTotal();
    *imgL = imread(lImgs[0]);
}

void panel::loadrightfile(QString path) {
    if (path.isEmpty())
        return;
    rImgs.clear();
    string file = QStr_to_string(path);
    getImgList(file, rImgs);
    *imgR = imread(rImgs[0]);
}

void panel::loadQMatrix(bool) {
    QString file = QFileDialog::getOpenFileName();
    string path = QStr_to_string(file);
    Q = loadQMat(path);
    ui->QFileShow->setText(file);
}

void panel::createLoadList() {
    loader = new LoadList(ui->loaderGroup);
    connect(loader, &LoadWidget::selectListFile, this, &panel::loadlistfile);
}

void panel::createLoadFile() {
    loader = new LoadFile(ui->loaderGroup);
    connect(loader, &LoadWidget::leftDir, this, &panel::loadleftfile);
    connect(loader, &LoadWidget::leftFile, this, &panel::loadleftfile);
    connect(loader, &LoadWidget::rightDir, this, &panel::loadrightfile);
    connect(loader, &LoadWidget::rightFile, this, &panel::loadrightfile);
}

void panel::updateCounter() {
    string msg = to_string(idx + 1);
    ui->index->setText(QString::fromUtf8(msg.c_str()));
}

/*******************************************************************************/
/*                           run group                                         */
/*******************************************************************************/

void panel::match(bool) {
    if (imgL->empty() || imgR->empty()) {
        QMessageBox msg;
        msg.setText("Missing image. Please select the image for left and right images.");
        msg.exec();
        return;
    }
    cloud->clear();
    updateCounter();
    dualView(*imgL, *imgR, "Image Pair");
    waitKey(10);
    *disp = alg->alg->match(*imgL, *imgR);
    *disp8 = showDisp(disp, alg->alg->getDispNum(), "Disparities");
}

void panel::on_previous_clicked(bool) {
    idx--;
    if(idx < 0)
        idx = lImgs.size() - 1;
    *imgL = imread(lImgs[idx]);
    *imgR = imread(rImgs[idx]);
    match(true);
}

void panel::on_next_clicked(bool) {
    idx++;
    if(idx >= lImgs.size())
        idx = 0;
    *imgL = imread(lImgs[idx]);
    *imgR = imread(rImgs[idx]);
    match(true);
}

void panel::on_cloud_clicked(bool) {
    if (Q.empty()) {
        QMessageBox box;
        box.setText("Please load Q Matrix from camera parameters.");
        box.exec();
        return;
    }
    if (cloud->empty())
        disp2cloud_rgb(disp, &Q, imgL, cloud);
    viewCloud(cloud);
}

/*******************************************************************************/
/*                              save group                                     */
/*******************************************************************************/
void panel::on_save_clicked(bool) {
    if (dispSavedPath.empty()) {
        QMessageBox msg;
        msg.setText("Please select a directory to save the matching result.");
        msg.exec();
        return;
    }
    if (ui->isSaveCloud->checkState() && cloudSavedPath.empty()) {
        QMessageBox msg;
        msg.setText("Missing the directory for saving cloud.");
        msg.exec();
        return;
    }
    fsys::path file(lImgs[idx]);
    fsys::path spath = dispSavedPath / file.stem();
    imwrite(spath.string() + "_disp.tiff", *disp);
    imwrite(spath.string() + "_disp8.png", *disp8);
    if (ui->isSaveCloud->checkState()) {
        if (cloud->empty()) {
            disp2cloud_rgb(disp, &Q, imgL, cloud);
        }
        saveCloud(cloud, cloudSavedPath.string(), file.string());
    }
}

void panel::selectDispSavePath(bool) {
    QString dir = QFileDialog::getExistingDirectory();
    ui->spShow->setText(dir);
    dispSavedPath = fsys::path(QStr_to_string(dir));
}

void panel::selectCloudSavePath(bool) {
    QString dir = QFileDialog::getExistingDirectory();
    ui->cloudPath->setText(dir);
    cloudSavedPath = fsys::path(QStr_to_string(dir));
}

void panel::on_saveCloud_checked(int state) {
    if (state == Qt::Checked) {
        if (Q.empty()) {
            QMessageBox box;
            box.setText("Please load Q Matrix from camera parameters.");
            box.exec();
        }
    }
}

/*******************************************************************************/
/*                         algorithm group                                     */
/*******************************************************************************/
void panel::on_sgbm(bool checked) {
    if(!checked)
        return;
    delete alg;
    alg = new sgbm_setting(ui->settingGroup);
    alg->updateSetting();
    alg->show();
}

void panel::on_bm(bool checked) {
    if(!checked)
        return;
    delete alg;
    alg = new bm_setting(ui->settingGroup);
    alg->updateSetting();
    alg->show();
}

void panel::on_elas(bool checked) {

}