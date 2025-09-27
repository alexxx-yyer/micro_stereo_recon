/********************************************************************************
** Form generated from reading UI file 'BM_Widget.ui'
**
** Created by: Qt User Interface Compiler version 6.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BM_H
#define UI_BM_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_bm_setting
{
public:
    QLabel *label;
    QSpinBox *BlockSize;
    QSpinBox *numDisp;
    QSpinBox *preFilterSize;
    QSpinBox *SpeckleRange;
    QSpinBox *uniquenessRatio;
    QSpinBox *SpeckleSize;
    QSpinBox *preFilterCap;
    QSpinBox *minDisp;
    QSpinBox *textureThreshold;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QSpinBox *disp12MaxDiff;
    QLabel *label_11;
    QComboBox *comboBox;

    void setupUi(QWidget *bm_setting)
    {
        if (bm_setting->objectName().isEmpty())
            bm_setting->setObjectName("bm_setting");
        bm_setting->resize(200, 270);
        label = new QLabel(bm_setting);
        label->setObjectName("label");
        label->setGeometry(QRect(5, 20, 95, 16));
        label->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        BlockSize = new QSpinBox(bm_setting);
        BlockSize->setObjectName("BlockSize");
        BlockSize->setGeometry(QRect(110, 20, 70, 16));
        BlockSize->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        numDisp = new QSpinBox(bm_setting);
        numDisp->setObjectName("numDisp");
        numDisp->setGeometry(QRect(110, 40, 70, 16));
        numDisp->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        preFilterSize = new QSpinBox(bm_setting);
        preFilterSize->setObjectName("preFilterSize");
        preFilterSize->setGeometry(QRect(110, 100, 70, 16));
        preFilterSize->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        SpeckleRange = new QSpinBox(bm_setting);
        SpeckleRange->setObjectName("SpeckleRange");
        SpeckleRange->setGeometry(QRect(110, 160, 70, 16));
        SpeckleRange->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        uniquenessRatio = new QSpinBox(bm_setting);
        uniquenessRatio->setObjectName("uniquenessRatio");
        uniquenessRatio->setGeometry(QRect(110, 200, 70, 16));
        uniquenessRatio->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        SpeckleSize = new QSpinBox(bm_setting);
        SpeckleSize->setObjectName("SpeckleSize");
        SpeckleSize->setGeometry(QRect(110, 140, 70, 16));
        SpeckleSize->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        preFilterCap = new QSpinBox(bm_setting);
        preFilterCap->setObjectName("preFilterCap");
        preFilterCap->setGeometry(QRect(110, 120, 70, 16));
        preFilterCap->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        minDisp = new QSpinBox(bm_setting);
        minDisp->setObjectName("minDisp");
        minDisp->setGeometry(QRect(110, 60, 70, 16));
        minDisp->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        textureThreshold = new QSpinBox(bm_setting);
        textureThreshold->setObjectName("textureThreshold");
        textureThreshold->setGeometry(QRect(110, 180, 70, 16));
        textureThreshold->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_2 = new QLabel(bm_setting);
        label_2->setObjectName("label_2");
        label_2->setGeometry(QRect(10, 40, 95, 16));
        label_2->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_3 = new QLabel(bm_setting);
        label_3->setObjectName("label_3");
        label_3->setGeometry(QRect(10, 60, 95, 16));
        label_3->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_4 = new QLabel(bm_setting);
        label_4->setObjectName("label_4");
        label_4->setGeometry(QRect(10, 80, 75, 16));
        label_4->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_5 = new QLabel(bm_setting);
        label_5->setObjectName("label_5");
        label_5->setGeometry(QRect(10, 100, 95, 16));
        label_5->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_6 = new QLabel(bm_setting);
        label_6->setObjectName("label_6");
        label_6->setGeometry(QRect(10, 120, 95, 16));
        label_6->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_7 = new QLabel(bm_setting);
        label_7->setObjectName("label_7");
        label_7->setGeometry(QRect(10, 140, 95, 16));
        label_7->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_8 = new QLabel(bm_setting);
        label_8->setObjectName("label_8");
        label_8->setGeometry(QRect(10, 160, 95, 16));
        label_8->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_9 = new QLabel(bm_setting);
        label_9->setObjectName("label_9");
        label_9->setGeometry(QRect(10, 180, 95, 16));
        label_9->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_10 = new QLabel(bm_setting);
        label_10->setObjectName("label_10");
        label_10->setGeometry(QRect(10, 200, 95, 16));
        label_10->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        disp12MaxDiff = new QSpinBox(bm_setting);
        disp12MaxDiff->setObjectName("disp12MaxDiff");
        disp12MaxDiff->setGeometry(QRect(110, 220, 70, 16));
        disp12MaxDiff->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_11 = new QLabel(bm_setting);
        label_11->setObjectName("label_11");
        label_11->setGeometry(QRect(10, 220, 95, 16));
        label_11->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        comboBox = new QComboBox(bm_setting);
        comboBox->setObjectName("comboBox");
        comboBox->setGeometry(QRect(85, 78, 95, 20));

        retranslateUi(bm_setting);

        QMetaObject::connectSlotsByName(bm_setting);
    } // setupUi

    void retranslateUi(QWidget *bm_setting)
    {
        bm_setting->setWindowTitle(QCoreApplication::translate("bm_setting", "Form", nullptr));
        label->setText(QCoreApplication::translate("bm_setting", "Block Size:", nullptr));
        label_2->setText(QCoreApplication::translate("bm_setting", "NumDisp:", nullptr));
        label_3->setText(QCoreApplication::translate("bm_setting", "MinDisp:", nullptr));
        label_4->setText(QCoreApplication::translate("bm_setting", "PreFilterType:", nullptr));
        label_5->setText(QCoreApplication::translate("bm_setting", "PreFilterSize:", nullptr));
        label_6->setText(QCoreApplication::translate("bm_setting", "PreFilterCap:", nullptr));
        label_7->setText(QCoreApplication::translate("bm_setting", "SpeckleSize:", nullptr));
        label_8->setText(QCoreApplication::translate("bm_setting", "SpeckleRange:", nullptr));
        label_9->setText(QCoreApplication::translate("bm_setting", "TextureThreshold:", nullptr));
        label_10->setText(QCoreApplication::translate("bm_setting", "UniquenessRatio:", nullptr));
        label_11->setText(QCoreApplication::translate("bm_setting", "Disp12MaxDiff:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class bm_setting: public Ui_bm_setting {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BM_H
