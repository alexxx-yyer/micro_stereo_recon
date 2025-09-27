/********************************************************************************
** Form generated from reading UI file 'panel.ui'
**
** Created by: Qt User Interface Compiler version 5.15.13
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PANEL_H
#define UI_PANEL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_panel
{
public:
    QGroupBox *groupBox;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QCheckBox *nintrinsics;
    QCheckBox *intrinsicsGuess;
    QHBoxLayout *horizontalLayout_2;
    QCheckBox *fixAspectRatio;
    QCheckBox *fixPrincipalPoint;
    QHBoxLayout *horizontalLayout_3;
    QCheckBox *fixFocalLength;
    QCheckBox *sameFocalLength;
    QHBoxLayout *horizontalLayout_4;
    QCheckBox *fixK1;
    QCheckBox *fixK2;
    QCheckBox *fixK3;
    QCheckBox *fixK4;
    QHBoxLayout *horizontalLayout_5;
    QCheckBox *fixK5;
    QCheckBox *fixK6;
    QCheckBox *fixS1S2S3S4;
    QHBoxLayout *horizontalLayout_6;
    QCheckBox *rationalModel;
    QCheckBox *thinPrismModel;
    QHBoxLayout *horizontalLayout_7;
    QCheckBox *tiltedModel;
    QCheckBox *fixTAUXTAUY;
    QHBoxLayout *horizontalLayout_8;
    QCheckBox *useQR;
    QCheckBox *useLU;
    QHBoxLayout *horizontalLayout_9;
    QCheckBox *fixTangentDist;
    QCheckBox *zeroTangentDist;
    QHBoxLayout *horizontalLayout_10;
    QCheckBox *fixIntrinsics;
    QCheckBox *zeroDisparity;
    QHBoxLayout *horizontalLayout_11;
    QCheckBox *extrinsicsGuess;
    QLabel *label_2;
    QHBoxLayout *horizontalLayout_12;
    QLabel *label_8;
    QSpinBox *iterNum;
    QLabel *label_9;
    QDoubleSpinBox *eps;
    QComboBox *precision;
    QGroupBox *groupBox_2;
    QLineEdit *lineEdit;
    QPushButton *loadFile;
    QLabel *label_3;
    QSpinBox *setRows;
    QSpinBox *setCols;
    QLabel *label_4;
    QLabel *label_5;
    QDoubleSpinBox *setInterval;
    QLabel *label_6;
    QPushButton *detectCorners;
    QPushButton *calculate;
    QLabel *label;
    QLabel *cornerNums;
    QLabel *label_7;
    QLabel *imgNums;
    QPushButton *save;
    QPushButton *saveCorners;
    QCheckBox *toGray;
    QPushButton *resetParams;

    void setupUi(QWidget *panel)
    {
        if (panel->objectName().isEmpty())
            panel->setObjectName(QString::fromUtf8("panel"));
        panel->resize(655, 382);
        groupBox = new QGroupBox(panel);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(20, 10, 321, 321));
        verticalLayoutWidget = new QWidget(groupBox);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 25, 300, 284));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(0);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        nintrinsics = new QCheckBox(verticalLayoutWidget);
        nintrinsics->setObjectName(QString::fromUtf8("nintrinsics"));

        horizontalLayout->addWidget(nintrinsics);

        intrinsicsGuess = new QCheckBox(verticalLayoutWidget);
        intrinsicsGuess->setObjectName(QString::fromUtf8("intrinsicsGuess"));
        intrinsicsGuess->setChecked(true);

        horizontalLayout->addWidget(intrinsicsGuess);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        fixAspectRatio = new QCheckBox(verticalLayoutWidget);
        fixAspectRatio->setObjectName(QString::fromUtf8("fixAspectRatio"));
        fixAspectRatio->setChecked(true);

        horizontalLayout_2->addWidget(fixAspectRatio);

        fixPrincipalPoint = new QCheckBox(verticalLayoutWidget);
        fixPrincipalPoint->setObjectName(QString::fromUtf8("fixPrincipalPoint"));

        horizontalLayout_2->addWidget(fixPrincipalPoint);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        fixFocalLength = new QCheckBox(verticalLayoutWidget);
        fixFocalLength->setObjectName(QString::fromUtf8("fixFocalLength"));

        horizontalLayout_3->addWidget(fixFocalLength);

        sameFocalLength = new QCheckBox(verticalLayoutWidget);
        sameFocalLength->setObjectName(QString::fromUtf8("sameFocalLength"));
        sameFocalLength->setChecked(true);

        horizontalLayout_3->addWidget(sameFocalLength);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        fixK1 = new QCheckBox(verticalLayoutWidget);
        fixK1->setObjectName(QString::fromUtf8("fixK1"));

        horizontalLayout_4->addWidget(fixK1);

        fixK2 = new QCheckBox(verticalLayoutWidget);
        fixK2->setObjectName(QString::fromUtf8("fixK2"));

        horizontalLayout_4->addWidget(fixK2);

        fixK3 = new QCheckBox(verticalLayoutWidget);
        fixK3->setObjectName(QString::fromUtf8("fixK3"));
        fixK3->setChecked(true);

        horizontalLayout_4->addWidget(fixK3);

        fixK4 = new QCheckBox(verticalLayoutWidget);
        fixK4->setObjectName(QString::fromUtf8("fixK4"));
        fixK4->setChecked(true);

        horizontalLayout_4->addWidget(fixK4);


        verticalLayout->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        fixK5 = new QCheckBox(verticalLayoutWidget);
        fixK5->setObjectName(QString::fromUtf8("fixK5"));
        fixK5->setChecked(true);

        horizontalLayout_5->addWidget(fixK5);

        fixK6 = new QCheckBox(verticalLayoutWidget);
        fixK6->setObjectName(QString::fromUtf8("fixK6"));

        horizontalLayout_5->addWidget(fixK6);

        fixS1S2S3S4 = new QCheckBox(verticalLayoutWidget);
        fixS1S2S3S4->setObjectName(QString::fromUtf8("fixS1S2S3S4"));

        horizontalLayout_5->addWidget(fixS1S2S3S4);


        verticalLayout->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        rationalModel = new QCheckBox(verticalLayoutWidget);
        rationalModel->setObjectName(QString::fromUtf8("rationalModel"));
        rationalModel->setChecked(true);

        horizontalLayout_6->addWidget(rationalModel);

        thinPrismModel = new QCheckBox(verticalLayoutWidget);
        thinPrismModel->setObjectName(QString::fromUtf8("thinPrismModel"));

        horizontalLayout_6->addWidget(thinPrismModel);


        verticalLayout->addLayout(horizontalLayout_6);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        tiltedModel = new QCheckBox(verticalLayoutWidget);
        tiltedModel->setObjectName(QString::fromUtf8("tiltedModel"));

        horizontalLayout_7->addWidget(tiltedModel);

        fixTAUXTAUY = new QCheckBox(verticalLayoutWidget);
        fixTAUXTAUY->setObjectName(QString::fromUtf8("fixTAUXTAUY"));

        horizontalLayout_7->addWidget(fixTAUXTAUY);


        verticalLayout->addLayout(horizontalLayout_7);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        useQR = new QCheckBox(verticalLayoutWidget);
        useQR->setObjectName(QString::fromUtf8("useQR"));

        horizontalLayout_8->addWidget(useQR);

        useLU = new QCheckBox(verticalLayoutWidget);
        useLU->setObjectName(QString::fromUtf8("useLU"));

        horizontalLayout_8->addWidget(useLU);


        verticalLayout->addLayout(horizontalLayout_8);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        fixTangentDist = new QCheckBox(verticalLayoutWidget);
        fixTangentDist->setObjectName(QString::fromUtf8("fixTangentDist"));

        horizontalLayout_9->addWidget(fixTangentDist);

        zeroTangentDist = new QCheckBox(verticalLayoutWidget);
        zeroTangentDist->setObjectName(QString::fromUtf8("zeroTangentDist"));
        zeroTangentDist->setChecked(true);

        horizontalLayout_9->addWidget(zeroTangentDist);


        verticalLayout->addLayout(horizontalLayout_9);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        fixIntrinsics = new QCheckBox(verticalLayoutWidget);
        fixIntrinsics->setObjectName(QString::fromUtf8("fixIntrinsics"));
        fixIntrinsics->setChecked(false);

        horizontalLayout_10->addWidget(fixIntrinsics);

        zeroDisparity = new QCheckBox(verticalLayoutWidget);
        zeroDisparity->setObjectName(QString::fromUtf8("zeroDisparity"));

        horizontalLayout_10->addWidget(zeroDisparity);


        verticalLayout->addLayout(horizontalLayout_10);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        extrinsicsGuess = new QCheckBox(verticalLayoutWidget);
        extrinsicsGuess->setObjectName(QString::fromUtf8("extrinsicsGuess"));

        horizontalLayout_11->addWidget(extrinsicsGuess);


        verticalLayout->addLayout(horizontalLayout_11);

        label_2 = new QLabel(verticalLayoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout->addWidget(label_2);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        label_8 = new QLabel(verticalLayoutWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout_12->addWidget(label_8);

        iterNum = new QSpinBox(verticalLayoutWidget);
        iterNum->setObjectName(QString::fromUtf8("iterNum"));
        iterNum->setMaximum(65535);
        iterNum->setValue(1000);

        horizontalLayout_12->addWidget(iterNum);

        label_9 = new QLabel(verticalLayoutWidget);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        horizontalLayout_12->addWidget(label_9);

        eps = new QDoubleSpinBox(verticalLayoutWidget);
        eps->setObjectName(QString::fromUtf8("eps"));
        eps->setValue(1.000000000000000);

        horizontalLayout_12->addWidget(eps);

        precision = new QComboBox(verticalLayoutWidget);
        precision->addItem(QString());
        precision->addItem(QString());
        precision->addItem(QString());
        precision->addItem(QString());
        precision->addItem(QString());
        precision->addItem(QString());
        precision->addItem(QString());
        precision->setObjectName(QString::fromUtf8("precision"));

        horizontalLayout_12->addWidget(precision);


        verticalLayout->addLayout(horizontalLayout_12);

        groupBox_2 = new QGroupBox(panel);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(360, 10, 261, 291));
        lineEdit = new QLineEdit(groupBox_2);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
        lineEdit->setGeometry(QRect(60, 20, 191, 20));
        lineEdit->setReadOnly(true);
        loadFile = new QPushButton(groupBox_2);
        loadFile->setObjectName(QString::fromUtf8("loadFile"));
        loadFile->setGeometry(QRect(10, 20, 41, 21));
        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 50, 91, 16));
        setRows = new QSpinBox(groupBox_2);
        setRows->setObjectName(QString::fromUtf8("setRows"));
        setRows->setGeometry(QRect(10, 70, 51, 16));
        setCols = new QSpinBox(groupBox_2);
        setCols->setObjectName(QString::fromUtf8("setCols"));
        setCols->setGeometry(QRect(120, 70, 51, 16));
        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(70, 70, 21, 16));
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(180, 70, 21, 16));
        setInterval = new QDoubleSpinBox(groupBox_2);
        setInterval->setObjectName(QString::fromUtf8("setInterval"));
        setInterval->setGeometry(QRect(100, 100, 61, 16));
        setInterval->setValue(1.000000000000000);
        label_6 = new QLabel(groupBox_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(60, 100, 40, 12));
        detectCorners = new QPushButton(groupBox_2);
        detectCorners->setObjectName(QString::fromUtf8("detectCorners"));
        detectCorners->setGeometry(QRect(30, 130, 61, 21));
        calculate = new QPushButton(groupBox_2);
        calculate->setObjectName(QString::fromUtf8("calculate"));
        calculate->setGeometry(QRect(140, 160, 61, 21));
        label = new QLabel(groupBox_2);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(40, 210, 40, 12));
        cornerNums = new QLabel(groupBox_2);
        cornerNums->setObjectName(QString::fromUtf8("cornerNums"));
        cornerNums->setGeometry(QRect(80, 210, 40, 12));
        label_7 = new QLabel(groupBox_2);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(120, 210, 40, 12));
        imgNums = new QLabel(groupBox_2);
        imgNums->setObjectName(QString::fromUtf8("imgNums"));
        imgNums->setGeometry(QRect(170, 210, 40, 12));
        save = new QPushButton(groupBox_2);
        save->setObjectName(QString::fromUtf8("save"));
        save->setGeometry(QRect(150, 250, 91, 21));
        saveCorners = new QPushButton(groupBox_2);
        saveCorners->setObjectName(QString::fromUtf8("saveCorners"));
        saveCorners->setGeometry(QRect(30, 250, 91, 21));
        toGray = new QCheckBox(groupBox_2);
        toGray->setObjectName(QString::fromUtf8("toGray"));
        toGray->setGeometry(QRect(120, 130, 111, 21));
        resetParams = new QPushButton(groupBox_2);
        resetParams->setObjectName(QString::fromUtf8("resetParams"));
        resetParams->setGeometry(QRect(30, 160, 61, 21));

        retranslateUi(panel);

        precision->setCurrentIndex(5);


        QMetaObject::connectSlotsByName(panel);
    } // setupUi

    void retranslateUi(QWidget *panel)
    {
        panel->setWindowTitle(QCoreApplication::translate("panel", "Stereo Calibration", nullptr));
        groupBox->setTitle(QCoreApplication::translate("panel", "\344\274\230\345\214\226\351\200\211\351\241\271", nullptr));
        nintrinsics->setText(QCoreApplication::translate("panel", "NINTRINSICS", nullptr));
        intrinsicsGuess->setText(QCoreApplication::translate("panel", "USE_INTRINSICS_GUESS", nullptr));
        fixAspectRatio->setText(QCoreApplication::translate("panel", "FIX_ASPECT_RATIO", nullptr));
        fixPrincipalPoint->setText(QCoreApplication::translate("panel", "FIX_PRINCIPAL_POINT", nullptr));
        fixFocalLength->setText(QCoreApplication::translate("panel", "FIX_FOCAL_LENGTH", nullptr));
        sameFocalLength->setText(QCoreApplication::translate("panel", "SAME_FOCAL_LENGTH", nullptr));
        fixK1->setText(QCoreApplication::translate("panel", "FIX_K1", nullptr));
        fixK2->setText(QCoreApplication::translate("panel", "FIX_K2", nullptr));
        fixK3->setText(QCoreApplication::translate("panel", "FIX_K3", nullptr));
        fixK4->setText(QCoreApplication::translate("panel", "FIX_K4", nullptr));
        fixK5->setText(QCoreApplication::translate("panel", "FIX_K5", nullptr));
        fixK6->setText(QCoreApplication::translate("panel", "FIX_K6", nullptr));
        fixS1S2S3S4->setText(QCoreApplication::translate("panel", "FIX_S1_S2_S3_S4", nullptr));
        rationalModel->setText(QCoreApplication::translate("panel", "RATIONAL_MODEL", nullptr));
        thinPrismModel->setText(QCoreApplication::translate("panel", "THIN_PRISM_MODEL", nullptr));
        tiltedModel->setText(QCoreApplication::translate("panel", "TILTED_MODEL", nullptr));
        fixTAUXTAUY->setText(QCoreApplication::translate("panel", "FIX_TAUX_TAUY", nullptr));
        useQR->setText(QCoreApplication::translate("panel", "USE_QR", nullptr));
        useLU->setText(QCoreApplication::translate("panel", "USE_LU", nullptr));
        fixTangentDist->setText(QCoreApplication::translate("panel", "FIX_TANGENT_DIST", nullptr));
        zeroTangentDist->setText(QCoreApplication::translate("panel", "ZERO_TANGENT_DIST", nullptr));
        fixIntrinsics->setText(QCoreApplication::translate("panel", "FIX_INTRINSICS", nullptr));
        zeroDisparity->setText(QCoreApplication::translate("panel", "ZERO_DISPARITY", nullptr));
        extrinsicsGuess->setText(QCoreApplication::translate("panel", "USE_EXTRINSICS_GUESS", nullptr));
        label_2->setText(QCoreApplication::translate("panel", "\347\273\210\346\255\242\346\235\241\344\273\266\357\274\232", nullptr));
        label_8->setText(QCoreApplication::translate("panel", "\350\277\255\344\273\243\346\254\241\346\225\260\357\274\232", nullptr));
        label_9->setText(QCoreApplication::translate("panel", "\347\262\276\345\272\246\357\274\232", nullptr));
        precision->setItemText(0, QCoreApplication::translate("panel", "e-1", nullptr));
        precision->setItemText(1, QCoreApplication::translate("panel", "e-2", nullptr));
        precision->setItemText(2, QCoreApplication::translate("panel", "e-3", nullptr));
        precision->setItemText(3, QCoreApplication::translate("panel", "e-4", nullptr));
        precision->setItemText(4, QCoreApplication::translate("panel", "e-5", nullptr));
        precision->setItemText(5, QCoreApplication::translate("panel", "e-6", nullptr));
        precision->setItemText(6, QCoreApplication::translate("panel", "e-7", nullptr));

        groupBox_2->setTitle(QCoreApplication::translate("panel", "\346\240\207\345\256\232", nullptr));
        loadFile->setText(QCoreApplication::translate("panel", "\346\226\207\344\273\266:", nullptr));
        label_3->setText(QCoreApplication::translate("panel", "\345\233\276\346\240\267\346\240\274\345\274\217\357\274\232", nullptr));
        label_4->setText(QCoreApplication::translate("panel", "\350\241\214", nullptr));
        label_5->setText(QCoreApplication::translate("panel", "\345\210\227", nullptr));
        label_6->setText(QCoreApplication::translate("panel", "\351\227\264\351\232\224\357\274\232", nullptr));
        detectCorners->setText(QCoreApplication::translate("panel", "\346\243\200\346\265\213\350\247\222\347\202\271", nullptr));
        calculate->setText(QCoreApplication::translate("panel", "\350\256\241\347\256\227\345\217\202\346\225\260", nullptr));
        label->setText(QCoreApplication::translate("panel", "\350\247\222\347\202\271\346\225\260\357\274\232", nullptr));
        cornerNums->setText(QCoreApplication::translate("panel", "--", nullptr));
        label_7->setText(QCoreApplication::translate("panel", "\345\233\276\347\211\207\345\257\271\357\274\232", nullptr));
        imgNums->setText(QCoreApplication::translate("panel", "--", nullptr));
        save->setText(QCoreApplication::translate("panel", "\344\277\235\345\255\230\346\240\207\345\256\232\347\273\223\346\236\234", nullptr));
        saveCorners->setText(QCoreApplication::translate("panel", "\344\277\235\345\255\230\350\247\222\347\202\271\345\233\276\347\211\207", nullptr));
        toGray->setText(QCoreApplication::translate("panel", "\351\246\226\345\205\210\350\275\254\344\270\272\347\201\260\345\272\246\345\233\276", nullptr));
        resetParams->setText(QCoreApplication::translate("panel", "\351\207\215\347\275\256\345\217\202\346\225\260", nullptr));
    } // retranslateUi

};

namespace Ui {
    class panel: public Ui_panel {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PANEL_H
