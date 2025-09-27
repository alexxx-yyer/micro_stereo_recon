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
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_sgbm_setting
{
public:
    QLabel *label;
    QSpinBox *BlockSize;
    QSpinBox *numDisp;
    QSpinBox *p1;
    QSpinBox *p2;
    QSpinBox *speckleSize;
    QSpinBox *disp12MaxDiff;
    QSpinBox *uniqueRatio;
    QSpinBox *preFilterCap;
    QSpinBox *minDisp;
    QSpinBox *speckleRange;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QComboBox *modeList;
    QLabel *label_11;

    void setupUi(QWidget *sgbm_setting)
    {
        if (sgbm_setting->objectName().isEmpty())
            sgbm_setting->setObjectName(QString::fromUtf8("sgbm_setting"));
        sgbm_setting->resize(200, 270);
        label = new QLabel(sgbm_setting);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(5, 20, 95, 16));
        label->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        BlockSize = new QSpinBox(sgbm_setting);
        BlockSize->setObjectName(QString::fromUtf8("BlockSize"));
        BlockSize->setGeometry(QRect(110, 20, 70, 16));
        BlockSize->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        BlockSize->setMaximum(1920);
        numDisp = new QSpinBox(sgbm_setting);
        numDisp->setObjectName(QString::fromUtf8("numDisp"));
        numDisp->setGeometry(QRect(110, 40, 70, 16));
        numDisp->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        numDisp->setMaximum(1920);
        p1 = new QSpinBox(sgbm_setting);
        p1->setObjectName(QString::fromUtf8("p1"));
        p1->setGeometry(QRect(110, 80, 70, 16));
        p1->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        p1->setMaximum(65535);
        p2 = new QSpinBox(sgbm_setting);
        p2->setObjectName(QString::fromUtf8("p2"));
        p2->setGeometry(QRect(110, 100, 70, 16));
        p2->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        p2->setMaximum(65535);
        speckleSize = new QSpinBox(sgbm_setting);
        speckleSize->setObjectName(QString::fromUtf8("speckleSize"));
        speckleSize->setGeometry(QRect(110, 160, 70, 16));
        speckleSize->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        disp12MaxDiff = new QSpinBox(sgbm_setting);
        disp12MaxDiff->setObjectName(QString::fromUtf8("disp12MaxDiff"));
        disp12MaxDiff->setGeometry(QRect(110, 200, 70, 16));
        disp12MaxDiff->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        uniqueRatio = new QSpinBox(sgbm_setting);
        uniqueRatio->setObjectName(QString::fromUtf8("uniqueRatio"));
        uniqueRatio->setGeometry(QRect(110, 140, 70, 16));
        uniqueRatio->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        preFilterCap = new QSpinBox(sgbm_setting);
        preFilterCap->setObjectName(QString::fromUtf8("preFilterCap"));
        preFilterCap->setGeometry(QRect(110, 120, 70, 16));
        preFilterCap->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        minDisp = new QSpinBox(sgbm_setting);
        minDisp->setObjectName(QString::fromUtf8("minDisp"));
        minDisp->setGeometry(QRect(110, 60, 70, 16));
        minDisp->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        minDisp->setMaximum(1920);
        speckleRange = new QSpinBox(sgbm_setting);
        speckleRange->setObjectName(QString::fromUtf8("speckleRange"));
        speckleRange->setGeometry(QRect(110, 180, 70, 16));
        speckleRange->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_2 = new QLabel(sgbm_setting);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(10, 40, 95, 16));
        label_2->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_3 = new QLabel(sgbm_setting);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 60, 95, 16));
        label_3->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_4 = new QLabel(sgbm_setting);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(10, 80, 95, 16));
        label_4->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_5 = new QLabel(sgbm_setting);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 100, 95, 16));
        label_5->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_6 = new QLabel(sgbm_setting);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(10, 120, 95, 16));
        label_6->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_7 = new QLabel(sgbm_setting);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(10, 140, 95, 16));
        label_7->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_8 = new QLabel(sgbm_setting);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(10, 160, 95, 16));
        label_8->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_9 = new QLabel(sgbm_setting);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(10, 180, 95, 16));
        label_9->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label_10 = new QLabel(sgbm_setting);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(10, 200, 95, 16));
        label_10->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        modeList = new QComboBox(sgbm_setting);
        modeList->setObjectName(QString::fromUtf8("modeList"));
        modeList->setGeometry(QRect(75, 220, 115, 20));
        label_11 = new QLabel(sgbm_setting);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(10, 220, 60, 20));
        label_11->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        retranslateUi(sgbm_setting);

        QMetaObject::connectSlotsByName(sgbm_setting);
    } // setupUi

    void retranslateUi(QWidget *sgbm_setting)
    {
        sgbm_setting->setWindowTitle(QCoreApplication::translate("sgbm_setting", "Form", nullptr));
        label->setText(QCoreApplication::translate("sgbm_setting", "Block Size:", nullptr));
        label_2->setText(QCoreApplication::translate("sgbm_setting", "NumDisp:", nullptr));
        label_3->setText(QCoreApplication::translate("sgbm_setting", "MinDisp:", nullptr));
        label_4->setText(QCoreApplication::translate("sgbm_setting", "P1:", nullptr));
        label_5->setText(QCoreApplication::translate("sgbm_setting", "P2:", nullptr));
        label_6->setText(QCoreApplication::translate("sgbm_setting", "PreFilterCap:", nullptr));
        label_7->setText(QCoreApplication::translate("sgbm_setting", "UniqueRatio:", nullptr));
        label_8->setText(QCoreApplication::translate("sgbm_setting", "SpeckleSize:", nullptr));
        label_9->setText(QCoreApplication::translate("sgbm_setting", "SpeckleRange:", nullptr));
        label_10->setText(QCoreApplication::translate("sgbm_setting", "Disp12MaxDiff:", nullptr));
        label_11->setText(QCoreApplication::translate("sgbm_setting", "Mode:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class sgbm_setting: public Ui_sgbm_setting {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SGBM_H
