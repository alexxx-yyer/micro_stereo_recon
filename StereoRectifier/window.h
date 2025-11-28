#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include "opencv2/core.hpp"

namespace Ui {
class window;
}

class window : public QWidget
{
    Q_OBJECT

public:
    explicit window(QWidget *parent = nullptr);
    ~window();

private:
    Ui::window *ui;
    int IFlag;
    cv::Size oldsize, newsize;
    cv::Mat M1, D1, M2, D2, R, T, R1, R2, P1, P2, Q;
    cv::Rect roi1, roi2;
    std::vector<std::string> lImgs, rImgs;

    inline void updateSizeGui();

private slots:
    void on_loadParams_clicked(bool checked);
    void on_loadImages_clicked(bool checked);
    void on_outputDir_clicked(bool checked);
    void on_rectify_clicked(bool checked);
    void on_imgWidth_editingFinished();
    void on_imgHeight_editingFinished();
    void on_showRectified_toggled(bool checked);
    void on_checkAlpha_toggled(bool checked);
    void on_presetSizes_currentIndexChanged(int index);
    void on_saveParams_clicked(bool checked);
};

#endif // WINDOW_H
