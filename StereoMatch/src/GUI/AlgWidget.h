#pragma once
#include <QWidget>
#include <QMessageBox>
#include <string>
#include "algorithms.h"
#include "ui_sgbm.h"
#include "ui_bm.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class AlgWidget;
}
QT_END_NAMESPACE

class AlgWidget : public QWidget
{
    Q_OBJECT
public:
    AlgWidget(QWidget *parent = nullptr) : QWidget(parent), alg(nullptr) {}
    virtual ~AlgWidget() {delete alg;}
    virtual void updateSetting() = 0;
    virtual void show() = 0;

    Alg_Impl *alg;
};

class sgbm_setting : public AlgWidget {
    Q_OBJECT
public:
    sgbm_setting(QWidget *parent) : AlgWidget(parent), ui(new Ui::sgbm_setting) {
        ui->setupUi(parent);
        alg = new Alg_SGBM_Impl;
        sgbm = (Alg_SGBM_Impl*) alg;
        ui->modeList->addItem("SGBM");
        ui->modeList->addItem("HH");
        ui->modeList->addItem("SGBM_3WAY");
        ui->modeList->addItem("HH4");

        connect(ui->BlockSize, &QSpinBox::valueChanged, this, &sgbm_setting::on_BlockSize_valueChanged);
        connect(ui->numDisp, &QSpinBox::valueChanged, this, &sgbm_setting::on_numDisp_valueChanged);
        connect(ui->minDisp, &QSpinBox::valueChanged, this, &sgbm_setting::on_minDisp_valueChanged);
        connect(ui->p1, &QSpinBox::valueChanged, this, &sgbm_setting::on_p1_valueChanged);
        connect(ui->p2, &QSpinBox::valueChanged, this, &sgbm_setting::on_p2_valueChanged);
        connect(ui->preFilterCap, &QSpinBox::valueChanged, this, &sgbm_setting::on_preFilterCap_valueChanged);
        connect(ui->uniqueRatio, &QSpinBox::valueChanged, this, &sgbm_setting::on_uniqueRatio_valueChanged);
        connect(ui->speckleSize, &QSpinBox::valueChanged, this, &sgbm_setting::on_speckleSize_valueChanged);
        connect(ui->speckleRange, &QSpinBox::valueChanged, this, &sgbm_setting::on_speckleRange_valueChanged);
        connect(ui->disp12MaxDiff, &QSpinBox::valueChanged, this, &sgbm_setting::on_disp12MaxDiff_valueChanged);
        connect(ui->modeList, &QComboBox::currentIndexChanged, this, &sgbm_setting::on_modeList_IndexChanged);
    }

    ~sgbm_setting() {
        delete ui->label;
        delete ui->label_2;
        delete ui->label_3;
        delete ui->label_4;
        delete ui->label_5;
        delete ui->label_6;
        delete ui->label_7;
        delete ui->label_8;
        delete ui->label_9;
        delete ui->label_10;
        delete ui->label_11;
        delete ui->BlockSize;
        delete ui->numDisp;
        delete ui->minDisp;
        delete ui->p1;
        delete ui->p2;
        delete ui->preFilterCap;
        delete ui->uniqueRatio;
        delete ui->speckleSize;
        delete ui->speckleRange;
        delete ui->disp12MaxDiff;
        delete ui->modeList;
        delete sgbm;
        alg = nullptr;
    }

    Ui::sgbm_setting *ui;

    void updateSetting() {
        ui->BlockSize->setValue(sgbm->alg->getBlockSize());
        ui->numDisp->setValue(sgbm->alg->getNumDisparities());
        ui->minDisp->setValue(sgbm->alg->getMinDisparity());
        ui->p1->setValue(sgbm->alg->getP1());
        ui->p2->setValue(sgbm->alg->getP2());
        ui->preFilterCap->setValue(sgbm->alg->getPreFilterCap());
        ui->uniqueRatio->setValue(sgbm->alg->getUniquenessRatio());
        ui->speckleSize->setValue(sgbm->alg->getSpeckleWindowSize());
        ui->speckleRange->setValue(sgbm->alg->getSpeckleRange());
        ui->disp12MaxDiff->setValue(sgbm->alg->getDisp12MaxDiff());
    }

    void show() {
        ui->label->show();
        ui->label_2->show();
        ui->label_3->show();
        ui->label_4->show();
        ui->label_5->show();
        ui->label_6->show();
        ui->label_7->show();
        ui->label_8->show();
        ui->label_9->show();
        ui->label_10->show();
        ui->BlockSize->show();
        ui->numDisp->show();
        ui->minDisp->show();
        ui->p1->show();
        ui->p2->show();
        ui->preFilterCap->show();
        ui->uniqueRatio->show();
        ui->speckleSize->show();
        ui->speckleRange->show();
        ui->disp12MaxDiff->show();
    }

private:
    Alg_SGBM_Impl *sgbm;

private slots:
    void on_BlockSize_valueChanged(int val) {sgbm->alg->setBlockSize(val);}
    void on_numDisp_valueChanged(int val) {sgbm->alg->setNumDisparities(val);}
    void on_minDisp_valueChanged(int val) {sgbm->alg->setMinDisparity(val);}
    void on_p1_valueChanged(int val) {sgbm->alg->setP1(val);}
    void on_p2_valueChanged(int val) {sgbm->alg->setP2(val);}
    void on_preFilterCap_valueChanged(int val) {sgbm->alg->setPreFilterCap(val);}
    void on_uniqueRatio_valueChanged(int val) {sgbm->alg->setUniquenessRatio(val);}
    void on_speckleSize_valueChanged(int val) {sgbm->alg->setSpeckleWindowSize(val);}
    void on_speckleRange_valueChanged(int val) {sgbm->alg->setSpeckleRange(val);}
    void on_disp12MaxDiff_valueChanged(int val) {sgbm->alg->setDisp12MaxDiff(val);}
    void on_modeList_IndexChanged(int idx) {sgbm->alg->setMode(idx);}
};

class bm_setting : public AlgWidget {
    Q_OBJECT
public:
    bm_setting(QWidget *parent) : AlgWidget(parent), ui(new Ui::bm_setting) {
        ui->setupUi(parent);
        ui->comboBox->addItem("NORMALIZED_RESPONSE");
        ui->comboBox->addItem("XSOBEL");
        bm = new Alg_BM_Impl;
        alg = bm;

        connect(ui->BlockSize, &QSpinBox::valueChanged, this, &bm_setting::on_BlockSize_changed);
        connect(ui->numDisp, &QSpinBox::valueChanged, this, &bm_setting::on_numDisp_changed);
        connect(ui->minDisp, &QSpinBox::valueChanged, this, &bm_setting::on_minDisp_changed);
        connect(ui->preFilterCap, &QSpinBox::valueChanged, this, &bm_setting::on_preFilterCap_changed);
        connect(ui->preFilterSize, &QSpinBox::valueChanged, this, &bm_setting::on_preFilterSize_changed);
        connect(ui->comboBox, &QComboBox::currentIndexChanged, this, &bm_setting::on_preFilterType_changed);
        connect(ui->SpeckleRange, &QSpinBox::valueChanged, this, &bm_setting::on_SpeckleRange_changed);
        connect(ui->SpeckleSize, &QSpinBox::valueChanged, this, &bm_setting::on_SpeckleSize_changed);
        connect(ui->textureThreshold, &QSpinBox::valueChanged, this, &bm_setting::on_textureThreshold_changed);
        connect(ui->uniquenessRatio, &QSpinBox::valueChanged, this, &bm_setting::on_uniquessRatio_changed);
        connect(ui->disp12MaxDiff, &QSpinBox::valueChanged, this, &bm_setting::on_disp12MaxDiff_changed);
    }

    ~bm_setting() {
        delete ui->label;
        delete ui->label_2;
        delete ui->label_3;
        delete ui->label_4;
        delete ui->label_5;
        delete ui->label_6;
        delete ui->label_7;
        delete ui->label_8;
        delete ui->label_9;
        delete ui->label_10;
        delete ui->label_11;
        delete ui->BlockSize;
        delete ui->numDisp;
        delete ui->preFilterSize;
        delete ui->SpeckleRange;
        delete ui->uniquenessRatio;
        delete ui->SpeckleSize;
        delete ui->preFilterCap;
        delete ui->minDisp;
        delete ui->textureThreshold;
        delete ui->disp12MaxDiff;
        delete ui->comboBox;
        delete ui;
        delete bm;
        alg = nullptr;
    };

    void updateSetting() {
        ui->BlockSize->setValue(bm->alg->getBlockSize());
        ui->numDisp->setValue(bm->alg->getNumDisparities());
        ui->preFilterCap->setValue(bm->alg->getPreFilterCap());
        ui->SpeckleRange->setValue(bm->alg->getSpeckleRange());
        ui->uniquenessRatio->setValue(bm->alg->getUniquenessRatio());
        ui->SpeckleSize->setValue(bm->alg->getSpeckleWindowSize());
        ui->preFilterSize->setValue(bm->alg->getPreFilterSize());
        ui->minDisp->setValue(bm->alg->getMinDisparity());
        ui->textureThreshold->setValue(bm->alg->getTextureThreshold());
        ui->disp12MaxDiff->setValue(bm->alg->getDisp12MaxDiff());
        ui->comboBox->setCurrentIndex(bm->alg->getPreFilterType());
    };

    void show() {
        ui->label->show();
        ui->label_2->show();
        ui->label_3->show();
        ui->label_4->show();
        ui->label_5->show();
        ui->label_6->show();
        ui->label_7->show();
        ui->label_8->show();
        ui->label_9->show();
        ui->label_10->show();
        ui->label_11->show();
        ui->BlockSize->show();
        ui->numDisp->show();
        ui->preFilterSize->show();
        ui->SpeckleRange->show();
        ui->uniquenessRatio->show();
        ui->SpeckleSize->show();
        ui->preFilterCap->show();
        ui->minDisp->show();
        ui->textureThreshold->show();
        ui->disp12MaxDiff->show();
        ui->comboBox->show();
    }

private:
    Ui::bm_setting *ui;
    Alg_BM_Impl *bm;

private slots:
    void on_BlockSize_changed(int val) {bm->alg->setBlockSize(val);}
    void on_numDisp_changed(int val) {bm->alg->setNumDisparities(val);}
    void on_minDisp_changed(int val) {bm->alg->setMinDisparity(val);}
    void on_preFilterCap_changed(int val) {bm->alg->setPreFilterCap(val);}
    void on_preFilterSize_changed(int val) {bm->alg->setPreFilterSize(val);}
    void on_preFilterType_changed(int val) {bm->alg->setPreFilterType(val);}
    void on_SpeckleRange_changed(int val) {bm->alg->setSpeckleRange(val);}
    void on_SpeckleSize_changed(int val) {bm->alg->setSpeckleWindowSize(val);}
    void on_uniquessRatio_changed(int val) {bm->alg->setUniquenessRatio(val);}
    void on_textureThreshold_changed(int val) {bm->alg->setTextureThreshold(val);}
    void on_disp12MaxDiff_changed(int val) {bm->alg->setDisp12MaxDiff(val);}
};