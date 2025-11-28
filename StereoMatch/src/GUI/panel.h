#pragma once
#include <QWidget>
#include "AlgWidget.h"
#include "LoadWidget.h"

#include "opencv2/core.hpp"
#include "pcl/io/pcd_io.h"
#include <filesystem>

QT_BEGIN_NAMESPACE
namespace Ui {
class panel;
}
QT_END_NAMESPACE

class panel : public QWidget
{
    Q_OBJECT
public:
    panel(QWidget *parent = nullptr);
    ~panel();

private:
    Ui::panel *ui;
    AlgWidget *alg;
    LoadWidget *loader;
    std::vector<std::string> lImgs, rImgs;
    size_t idx;
    cv::Mat *disp, *disp8;
    cv::Mat *imgL, *imgR;
    cv::Mat Q;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    std::filesystem::path dispSavedPath;
    std::filesystem::path cloudSavedPath;

    void createLoadList();
    void createLoadFile();
    void updateTotal();
    inline void updateCounter();


public slots:
    // loader group
    void on_loadlist(bool checked);
    void on_loadfile(bool checked);
    void loadlistfile(QString path);
    void loadleftfile(QString path);
    void loadrightfile(QString path);
    void loadQMatrix(bool checked);

    // run group
    void match(bool checked);
    void on_previous_clicked(bool checked);
    void on_next_clicked(bool checked);
    void on_cloud_clicked(bool checked);

    // save group
    void on_save_clicked(bool checked);
    void selectDispSavePath(bool checked);
    void selectCloudSavePath(bool checked);
    void on_saveCloud_checked(int state);

    // algorithm group
    void on_sgbm(bool checked);
    void on_bm(bool checked);
    void on_elas(bool checked);
};