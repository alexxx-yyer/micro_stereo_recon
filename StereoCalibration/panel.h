#ifndef PANEL_H
#define PANEL_H

#include <QWidget>
#include "opencv2/core.hpp"

// 声明命名空间Ui，用于存放UI相关的类
namespace Ui {
class panel;
}

// panel类继承自QWidget，用于创建一个面板窗口
class panel : public QWidget
{
    Q_OBJECT

public:
    // 构造函数，explicit关键字防止隐式转换，parent为父窗口指针
    explicit panel(QWidget *parent = nullptr);
    // 析构函数
    ~panel();

private:
    // 指向UI类的指针，用于访问UI组件
    Ui::panel *ui;

    // 存储左图和右图文件路径的向量
    std::vector<std::string> lImgs, rImgs;
    // 存储坏点索引的向量
    std::vector<int> badIdx;
    // 存储左图和右图角点的二维向量
    std::vector<std::vector<cv::Point2f>> corners1, corners2;
    // 存储物体点的三维坐标的向量
    std::vector<cv::Point3f> objPoints;
    // 标志变量，用于控制不同的参数设置
    int flag;
    // 图像尺寸
    cv::Size imgsize;
    // 终止准则，用于迭代算法的终止条件
    cv::TermCriteria criteria;
    // 相机内参矩阵和畸变系数
    cv::Mat M1, D1, M2, D2, R, T, E, F, perError;

    // 内联函数，用于修改标志变量
    inline void modifyFlag(int sign, bool checked);
    // 获取精度的函数
    double getPrecision();

private slots:
    // 加载文件按钮点击事件处理函数
    void on_loadFile_clicked(bool checked);
    // 初始化物体点函数
    void init_objPoints();
    // 检测角点按钮点击事件处理函数
    void on_detectCorners_clicked(bool checked);
    // 各个参数设置按钮的点击事件处理函数
    void on_nintrinsics_toggled(bool checked);
    void on_intrinsicsGuess_toggled(bool checked);
    void on_fixAspectRatio_toggled(bool checked);
    void on_fixPrincipalPoint_toggled(bool checked);
    void on_fixFocalLength_toggled(bool checked);
    void on_sameFocalLength_toggled(bool checked);
    void on_fixK1_toggled(bool checked);
    void on_fixK2_toggled(bool checked);
    void on_fixK3_toggled(bool checked);
    void on_fixK4_toggled(bool checked);
    void on_fixK5_toggled(bool checked);
    void on_fixK6_toggled(bool checked);
    void on_fixS1S2S3S4_toggled(bool checked);
    void on_rationalModel_toggled(bool checked);
    void on_thinPrismModel_toggled(bool checked);
    void on_tiltedModel_toggled(bool checked);
    void on_fixTAUXTAUY_toggled(bool checked);
    void on_useQR_toggled(bool checked);
    void on_useLU_toggled(bool checked);
    void on_fixTangentDist_toggled(bool checked);
    void on_zeroTangentDist_toggled(bool checked);
    void on_fixIntrinsics_toggled(bool checked);
    void on_zeroDisparity_toggled(bool checked);
    void on_extrinsicsGuess_toggled(bool checked);
    // 迭代次数编辑完成事件处理函数
    void on_iterNum_editingFinished();
    // 精度编辑完成事件处理函数
    void on_eps_editingFinished();
    // 精度选择框当前索引改变事件处理函数
    void on_precision_currentIndexChanged(int index);
    // 计算按钮点击事件处理函数
    void on_calculate_clicked(bool checked);
    // 重置参数按钮点击事件处理函数
    void on_resetParams_clicked(bool checked);
    // 保存按钮点击事件处理函数
    void on_save_clicked(bool checked);
    // 保存角点按钮点击事件处理函数
    void on_saveCorners_clicked(bool checked);
};

#endif // PANEL_H