/********************************************************************************
** Form generated from reading UI file 'SGBM_Widget.ui'
**
** Created by: Qt User Interface Compiler version 5.15.13
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SGBM_H
#define UI_SGBM_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_sgbm_setting
{
public:
    QLabel *label;
    QSpinBox *BlockSize;
    QLabel *label_3;
    QSpinBox *minDisp;
    QLabel *label_4;
    QSpinBox *p1;
    QLabel *label_5;
    QSpinBox *p2;
    QLabel *label_6;
    QSpinBox *preFilterCap;
    QLabel *label_7;
    QSpinBox *uniqueRatio;
    QLabel *label_8;
    QSpinBox *speckleSize;
    QLabel *label_9;
    QSpinBox *speckleRange;
    QLabel *label_10;
    QSpinBox *disp12MaxDiff;
    QLabel *label_2;
    QSpinBox *numDisp;
    QLabel *label_11;
    QComboBox *modeList;
    QCheckBox *useWLS;
    QLabel *label_12;
    QDoubleSpinBox *wlsLambda;
    QLabel *label_13;
    QDoubleSpinBox *wlsSigma;
    QCheckBox *useFBS;
    QLabel *label_14;
    QDoubleSpinBox *fbsLambda;
    QLabel *label_15;
    QDoubleSpinBox *fbsSigma;

    void setupUi(QWidget *sgbm_setting)
    {
        if (sgbm_setting->objectName().isEmpty())
            sgbm_setting->setObjectName(QString::fromUtf8("sgbm_setting"));
        sgbm_setting->resize(350, 420);
        label = new QLabel(sgbm_setting);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 20, 100, 20));
        label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BlockSize = new QSpinBox(sgbm_setting);
        BlockSize->setObjectName(QString::fromUtf8("BlockSize"));
        BlockSize->setGeometry(QRect(120, 20, 80, 20));
        BlockSize->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BlockSize->setMaximum(1920);
        label_3 = new QLabel(sgbm_setting);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 50, 100, 20));
        label_3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        minDisp = new QSpinBox(sgbm_setting);
        minDisp->setObjectName(QString::fromUtf8("minDisp"));
        minDisp->setGeometry(QRect(120, 50, 80, 20));
        minDisp->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_4 = new QLabel(sgbm_setting);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(10, 80, 100, 20));
        label_4->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        p1 = new QSpinBox(sgbm_setting);
        p1->setObjectName(QString::fromUtf8("p1"));
        p1->setGeometry(QRect(120, 80, 80, 20));
        p1->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_5 = new QLabel(sgbm_setting);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 110, 100, 20));
        label_5->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        p2 = new QSpinBox(sgbm_setting);
        p2->setObjectName(QString::fromUtf8("p2"));
        p2->setGeometry(QRect(120, 110, 80, 20));
        p2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_6 = new QLabel(sgbm_setting);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(10, 140, 100, 20));
        label_6->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        preFilterCap = new QSpinBox(sgbm_setting);
        preFilterCap->setObjectName(QString::fromUtf8("preFilterCap"));
        preFilterCap->setGeometry(QRect(120, 140, 80, 20));
        preFilterCap->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_7 = new QLabel(sgbm_setting);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(10, 170, 100, 20));
        label_7->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        uniqueRatio = new QSpinBox(sgbm_setting);
        uniqueRatio->setObjectName(QString::fromUtf8("uniqueRatio"));
        uniqueRatio->setGeometry(QRect(120, 170, 80, 20));
        uniqueRatio->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_8 = new QLabel(sgbm_setting);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(10, 200, 100, 20));
        label_8->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        speckleSize = new QSpinBox(sgbm_setting);
        speckleSize->setObjectName(QString::fromUtf8("speckleSize"));
        speckleSize->setGeometry(QRect(120, 200, 80, 20));
        speckleSize->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_9 = new QLabel(sgbm_setting);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(10, 230, 100, 20));
        label_9->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        speckleRange = new QSpinBox(sgbm_setting);
        speckleRange->setObjectName(QString::fromUtf8("speckleRange"));
        speckleRange->setGeometry(QRect(120, 230, 80, 20));
        speckleRange->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_10 = new QLabel(sgbm_setting);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(10, 260, 100, 20));
        label_10->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        disp12MaxDiff = new QSpinBox(sgbm_setting);
        disp12MaxDiff->setObjectName(QString::fromUtf8("disp12MaxDiff"));
        disp12MaxDiff->setGeometry(QRect(120, 260, 80, 20));
        disp12MaxDiff->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_2 = new QLabel(sgbm_setting);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(220, 20, 100, 20));
        label_2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        numDisp = new QSpinBox(sgbm_setting);
        numDisp->setObjectName(QString::fromUtf8("numDisp"));
        numDisp->setGeometry(QRect(330, 20, 80, 20));
        numDisp->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        numDisp->setMaximum(1024);
        label_11 = new QLabel(sgbm_setting);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(220, 50, 60, 20));
        label_11->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        modeList = new QComboBox(sgbm_setting);
        modeList->setObjectName(QString::fromUtf8("modeList"));
        modeList->setGeometry(QRect(290, 50, 120, 20));
        useWLS = new QCheckBox(sgbm_setting);
        useWLS->setObjectName(QString::fromUtf8("useWLS"));
        useWLS->setGeometry(QRect(220, 80, 120, 20));
        useWLS->setChecked(true);
        label_12 = new QLabel(sgbm_setting);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(220, 110, 80, 20));
        label_12->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        wlsLambda = new QDoubleSpinBox(sgbm_setting);
        wlsLambda->setObjectName(QString::fromUtf8("wlsLambda"));
        wlsLambda->setGeometry(QRect(310, 110, 100, 20));
        wlsLambda->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        wlsLambda->setDecimals(1);
        wlsLambda->setMaximum(50000.000000000000000);
        wlsLambda->setMinimum(1.000000000000000);
        wlsLambda->setSingleStep(100.000000000000000);
        wlsLambda->setValue(8000.000000000000000);
        label_13 = new QLabel(sgbm_setting);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setGeometry(QRect(220, 140, 80, 20));
        label_13->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        wlsSigma = new QDoubleSpinBox(sgbm_setting);
        wlsSigma->setObjectName(QString::fromUtf8("wlsSigma"));
        wlsSigma->setGeometry(QRect(310, 140, 100, 20));
        wlsSigma->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        wlsSigma->setDecimals(2);
        wlsSigma->setMaximum(10.000000000000000);
        wlsSigma->setMinimum(0.100000000000000);
        wlsSigma->setSingleStep(0.100000000000000);
        wlsSigma->setValue(1.500000000000000);
        useFBS = new QCheckBox(sgbm_setting);
        useFBS->setObjectName(QString::fromUtf8("useFBS"));
        useFBS->setGeometry(QRect(220, 170, 120, 20));
        useFBS->setChecked(false);
        label_14 = new QLabel(sgbm_setting);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(220, 200, 80, 20));
        label_14->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        fbsLambda = new QDoubleSpinBox(sgbm_setting);
        fbsLambda->setObjectName(QString::fromUtf8("fbsLambda"));
        fbsLambda->setGeometry(QRect(310, 200, 100, 20));
        fbsLambda->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        fbsLambda->setDecimals(1);
        fbsLambda->setMaximum(1000.000000000000000);
        fbsLambda->setMinimum(1.000000000000000);
        fbsLambda->setSingleStep(10.000000000000000);
        fbsLambda->setValue(128.000000000000000);
        label_15 = new QLabel(sgbm_setting);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(220, 230, 80, 20));
        label_15->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        fbsSigma = new QDoubleSpinBox(sgbm_setting);
        fbsSigma->setObjectName(QString::fromUtf8("fbsSigma"));
        fbsSigma->setGeometry(QRect(310, 230, 100, 20));
        fbsSigma->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        fbsSigma->setDecimals(3);
        fbsSigma->setMaximum(1.000000000000000);
        fbsSigma->setMinimum(0.001000000000000);
        fbsSigma->setSingleStep(0.010000000000000);
        fbsSigma->setValue(0.050000000000000);

        retranslateUi(sgbm_setting);

        QMetaObject::connectSlotsByName(sgbm_setting);
    } // setupUi

    void retranslateUi(QWidget *sgbm_setting)
    {
        sgbm_setting->setWindowTitle(QCoreApplication::translate("sgbm_setting", "SGBM Parameters", nullptr));
        label->setText(QCoreApplication::translate("sgbm_setting", "Block Size:", nullptr));
        label_3->setText(QCoreApplication::translate("sgbm_setting", "Min Disparity:", nullptr));
        label_4->setText(QCoreApplication::translate("sgbm_setting", "P1:", nullptr));
        label_5->setText(QCoreApplication::translate("sgbm_setting", "P2:", nullptr));
        label_6->setText(QCoreApplication::translate("sgbm_setting", "PreFilter Cap:", nullptr));
        label_7->setText(QCoreApplication::translate("sgbm_setting", "Unique Ratio:", nullptr));
        label_8->setText(QCoreApplication::translate("sgbm_setting", "Speckle Size:", nullptr));
        label_9->setText(QCoreApplication::translate("sgbm_setting", "Speckle Range:", nullptr));
        label_10->setText(QCoreApplication::translate("sgbm_setting", "Disp12MaxDiff:", nullptr));
        label_2->setText(QCoreApplication::translate("sgbm_setting", "Num Disparities:", nullptr));
        label_11->setText(QCoreApplication::translate("sgbm_setting", "Mode:", nullptr));
        useWLS->setText(QCoreApplication::translate("sgbm_setting", "Use WLS Filter", nullptr));
        label_12->setText(QCoreApplication::translate("sgbm_setting", "WLS Lambda:", nullptr));
        label_13->setText(QCoreApplication::translate("sgbm_setting", "WLS Sigma:", nullptr));
        useFBS->setText(QCoreApplication::translate("sgbm_setting", "Use FBS Filter", nullptr));
        label_14->setText(QCoreApplication::translate("sgbm_setting", "FBS Lambda:", nullptr));
        label_15->setText(QCoreApplication::translate("sgbm_setting", "FBS Sigma:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class sgbm_setting: public Ui_sgbm_setting {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SGBM_H
