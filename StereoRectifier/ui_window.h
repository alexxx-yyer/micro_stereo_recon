/********************************************************************************
** Form generated from reading UI file 'window.ui'
**
** Created by: Qt User Interface Compiler version 5.15.13
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WINDOW_H
#define UI_WINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_window
{
public:
    QPushButton *loadParams;
    QPushButton *loadImages;
    QLineEdit *lineEdit;
    QLineEdit *lineEdit_2;
    QPushButton *rectify;
    QPushButton *outputDir;
    QLineEdit *lineEdit_3;
    QLabel *label;
    QCheckBox *calibZeroDisp;
    QCheckBox *checkAlpha;
    QDoubleSpinBox *alpha;
    QLineEdit *imgWidth;
    QLabel *label_2;
    QLineEdit *imgHeight;
    QComboBox *presetSizes;
    QCheckBox *showRectified;
    QCheckBox *manualSave;
    QPushButton *saveParams;

    void setupUi(QWidget *window)
    {
        if (window->objectName().isEmpty())
            window->setObjectName(QString::fromUtf8("window"));
        window->resize(400, 300);
        loadParams = new QPushButton(window);
        loadParams->setObjectName(QString::fromUtf8("loadParams"));
        loadParams->setGeometry(QRect(30, 20, 75, 23));
        loadImages = new QPushButton(window);
        loadImages->setObjectName(QString::fromUtf8("loadImages"));
        loadImages->setGeometry(QRect(30, 60, 75, 23));
        lineEdit = new QLineEdit(window);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
        lineEdit->setGeometry(QRect(120, 20, 241, 20));
        lineEdit_2 = new QLineEdit(window);
        lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));
        lineEdit_2->setGeometry(QRect(120, 60, 241, 20));
        rectify = new QPushButton(window);
        rectify->setObjectName(QString::fromUtf8("rectify"));
        rectify->setGeometry(QRect(60, 230, 75, 23));
        outputDir = new QPushButton(window);
        outputDir->setObjectName(QString::fromUtf8("outputDir"));
        outputDir->setGeometry(QRect(30, 100, 75, 23));
        lineEdit_3 = new QLineEdit(window);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));
        lineEdit_3->setGeometry(QRect(120, 100, 241, 20));
        label = new QLabel(window);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(40, 150, 61, 21));
        calibZeroDisp = new QCheckBox(window);
        calibZeroDisp->setObjectName(QString::fromUtf8("calibZeroDisp"));
        calibZeroDisp->setGeometry(QRect(40, 200, 171, 16));
        checkAlpha = new QCheckBox(window);
        checkAlpha->setObjectName(QString::fromUtf8("checkAlpha"));
        checkAlpha->setGeometry(QRect(220, 200, 61, 16));
        alpha = new QDoubleSpinBox(window);
        alpha->setObjectName(QString::fromUtf8("alpha"));
        alpha->setGeometry(QRect(290, 200, 71, 16));
        alpha->setMaximum(1.000000000000000);
        alpha->setSingleStep(0.100000000000000);
        imgWidth = new QLineEdit(window);
        imgWidth->setObjectName(QString::fromUtf8("imgWidth"));
        imgWidth->setGeometry(QRect(100, 150, 61, 21));
        label_2 = new QLabel(window);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(170, 150, 16, 21));
        imgHeight = new QLineEdit(window);
        imgHeight->setObjectName(QString::fromUtf8("imgHeight"));
        imgHeight->setGeometry(QRect(190, 150, 61, 21));
        presetSizes = new QComboBox(window);
        presetSizes->addItem(QString());
        presetSizes->addItem(QString());
        presetSizes->setObjectName(QString::fromUtf8("presetSizes"));
        presetSizes->setGeometry(QRect(270, 150, 91, 21));
        showRectified = new QCheckBox(window);
        showRectified->setObjectName(QString::fromUtf8("showRectified"));
        showRectified->setGeometry(QRect(160, 230, 71, 23));
        manualSave = new QCheckBox(window);
        manualSave->setObjectName(QString::fromUtf8("manualSave"));
        manualSave->setGeometry(QRect(250, 230, 71, 23));
        saveParams = new QPushButton(window);
        saveParams->setObjectName(QString::fromUtf8("saveParams"));
        saveParams->setGeometry(QRect(150, 260, 91, 23));

        retranslateUi(window);

        QMetaObject::connectSlotsByName(window);
    } // setupUi

    void retranslateUi(QWidget *window)
    {
        window->setWindowTitle(QCoreApplication::translate("window", "Form", nullptr));
        loadParams->setText(QCoreApplication::translate("window", "\347\233\270\346\234\272\345\217\202\346\225\260", nullptr));
        loadImages->setText(QCoreApplication::translate("window", "\350\276\223\345\205\245\345\233\276\347\211\207", nullptr));
        rectify->setText(QCoreApplication::translate("window", "\346\236\201\347\272\277\346\240\241\346\255\243", nullptr));
        outputDir->setText(QCoreApplication::translate("window", "\350\276\223\345\207\272\347\233\256\345\275\225", nullptr));
        label->setText(QCoreApplication::translate("window", "\347\233\256\346\240\207\345\260\272\345\257\270:", nullptr));
        calibZeroDisp->setText(QCoreApplication::translate("window", "CALIB_ZERO_DISPARITY", nullptr));
        checkAlpha->setText(QCoreApplication::translate("window", "alpha", nullptr));
        label_2->setText(QCoreApplication::translate("window", "x", nullptr));
        presetSizes->setItemText(0, QCoreApplication::translate("window", "As Input", nullptr));
        presetSizes->setItemText(1, QCoreApplication::translate("window", "Custom", nullptr));

        showRectified->setText(QCoreApplication::translate("window", "\345\261\225\347\244\272\347\273\223\346\236\234", nullptr));
        manualSave->setText(QCoreApplication::translate("window", "\346\211\213\345\212\250\344\277\235\345\255\230", nullptr));
        saveParams->setText(QCoreApplication::translate("window", "\344\277\235\345\255\230\346\240\241\346\255\243\345\217\202\346\225\260", nullptr));
    } // retranslateUi

};

namespace Ui {
    class window: public Ui_window {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WINDOW_H
