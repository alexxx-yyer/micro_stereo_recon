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
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_panel
{
public:
    QGroupBox *settingGroup;
    QGroupBox *runGroup;
    QPushButton *runButton;
    QPushButton *nextButton;
    QPushButton *previousButton;
    QPushButton *cloudButton;
    QGroupBox *loaderGroup;
    QLabel *QMLabel;
    QLabel *QFileShow;
    QPushButton *QFile;
    QFrame *line;
    QRadioButton *listMode;
    QRadioButton *dirMode;
    QLabel *index;
    QLabel *total;
    QGroupBox *AlgorithmsGroup;
    QRadioButton *SGBMButton;
    QRadioButton *ELASButton;
    QRadioButton *BMButton;
    QGroupBox *saveGroup;
    QPushButton *cloudPathButton;
    QLabel *cloudPath;
    QLabel *cloudLabel;
    QPushButton *savePathButton;
    QLabel *spShow;
    QLabel *spLabel;
    QPushButton *saveButton;
    QCheckBox *isSaveCloud;

    void setupUi(QWidget *panel)
    {
        if (panel->objectName().isEmpty())
            panel->setObjectName(QString::fromUtf8("panel"));
        panel->resize(620, 300);
        settingGroup = new QGroupBox(panel);
        settingGroup->setObjectName(QString::fromUtf8("settingGroup"));
        settingGroup->setGeometry(QRect(20, 10, 200, 270));
        runGroup = new QGroupBox(panel);
        runGroup->setObjectName(QString::fromUtf8("runGroup"));
        runGroup->setGeometry(QRect(230, 85, 175, 70));
        runButton = new QPushButton(runGroup);
        runButton->setObjectName(QString::fromUtf8("runButton"));
        runButton->setGeometry(QRect(20, 40, 60, 25));
        nextButton = new QPushButton(runGroup);
        nextButton->setObjectName(QString::fromUtf8("nextButton"));
        nextButton->setGeometry(QRect(100, 15, 60, 25));
        previousButton = new QPushButton(runGroup);
        previousButton->setObjectName(QString::fromUtf8("previousButton"));
        previousButton->setGeometry(QRect(20, 15, 60, 25));
        cloudButton = new QPushButton(runGroup);
        cloudButton->setObjectName(QString::fromUtf8("cloudButton"));
        cloudButton->setGeometry(QRect(100, 40, 60, 25));
        loaderGroup = new QGroupBox(panel);
        loaderGroup->setObjectName(QString::fromUtf8("loaderGroup"));
        loaderGroup->setGeometry(QRect(415, 10, 185, 270));
        QMLabel = new QLabel(loaderGroup);
        QMLabel->setObjectName(QString::fromUtf8("QMLabel"));
        QMLabel->setGeometry(QRect(10, 222, 60, 12));
        QFileShow = new QLabel(loaderGroup);
        QFileShow->setObjectName(QString::fromUtf8("QFileShow"));
        QFileShow->setGeometry(QRect(10, 234, 40, 12));
        QFileShow->setAlignment(Qt::AlignmentFlag::AlignLeading|Qt::AlignmentFlag::AlignLeft|Qt::AlignmentFlag::AlignTop);
        QFile = new QPushButton(loaderGroup);
        QFile->setObjectName(QString::fromUtf8("QFile"));
        QFile->setGeometry(QRect(160, 230, 20, 20));
        QIcon icon(QIcon::fromTheme(QString::fromUtf8("QIcon::ThemeIcon::DocumentNew")));
        QFile->setIcon(icon);
        line = new QFrame(loaderGroup);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(10, 190, 165, 20));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        listMode = new QRadioButton(loaderGroup);
        listMode->setObjectName(QString::fromUtf8("listMode"));
        listMode->setGeometry(QRect(15, 20, 70, 16));
        dirMode = new QRadioButton(loaderGroup);
        dirMode->setObjectName(QString::fromUtf8("dirMode"));
        dirMode->setGeometry(QRect(100, 20, 80, 16));
        dirMode->setChecked(true);
        index = new QLabel(loaderGroup);
        index->setObjectName(QString::fromUtf8("index"));
        index->setGeometry(QRect(60, 180, 21, 16));
        index->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        total = new QLabel(loaderGroup);
        total->setObjectName(QString::fromUtf8("total"));
        total->setGeometry(QRect(81, 180, 53, 16));
        QFont font;
        font.setPointSize(9);
        total->setFont(font);
        AlgorithmsGroup = new QGroupBox(panel);
        AlgorithmsGroup->setObjectName(QString::fromUtf8("AlgorithmsGroup"));
        AlgorithmsGroup->setGeometry(QRect(230, 10, 175, 70));
        SGBMButton = new QRadioButton(AlgorithmsGroup);
        SGBMButton->setObjectName(QString::fromUtf8("SGBMButton"));
        SGBMButton->setGeometry(QRect(20, 20, 72, 16));
        SGBMButton->setIconSize(QSize(16, 16));
        SGBMButton->setChecked(true);
        ELASButton = new QRadioButton(AlgorithmsGroup);
        ELASButton->setObjectName(QString::fromUtf8("ELASButton"));
        ELASButton->setGeometry(QRect(20, 40, 72, 16));
        BMButton = new QRadioButton(AlgorithmsGroup);
        BMButton->setObjectName(QString::fromUtf8("BMButton"));
        BMButton->setGeometry(QRect(90, 20, 72, 16));
        saveGroup = new QGroupBox(panel);
        saveGroup->setObjectName(QString::fromUtf8("saveGroup"));
        saveGroup->setGeometry(QRect(230, 160, 175, 120));
        cloudPathButton = new QPushButton(saveGroup);
        cloudPathButton->setObjectName(QString::fromUtf8("cloudPathButton"));
        cloudPathButton->setGeometry(QRect(150, 87, 20, 20));
        QIcon icon1(QIcon::fromTheme(QString::fromUtf8("QIcon::ThemeIcon::FolderOpen")));
        cloudPathButton->setIcon(icon1);
        cloudPath = new QLabel(saveGroup);
        cloudPath->setObjectName(QString::fromUtf8("cloudPath"));
        cloudPath->setGeometry(QRect(5, 91, 145, 24));
        cloudPath->setAlignment(Qt::AlignmentFlag::AlignLeading|Qt::AlignmentFlag::AlignLeft|Qt::AlignmentFlag::AlignTop);
        cloudLabel = new QLabel(saveGroup);
        cloudLabel->setObjectName(QString::fromUtf8("cloudLabel"));
        cloudLabel->setGeometry(QRect(5, 79, 80, 15));
        cloudLabel->setAlignment(Qt::AlignmentFlag::AlignLeading|Qt::AlignmentFlag::AlignLeft|Qt::AlignmentFlag::AlignVCenter);
        savePathButton = new QPushButton(saveGroup);
        savePathButton->setObjectName(QString::fromUtf8("savePathButton"));
        savePathButton->setGeometry(QRect(150, 50, 20, 20));
        savePathButton->setIcon(icon1);
        spShow = new QLabel(saveGroup);
        spShow->setObjectName(QString::fromUtf8("spShow"));
        spShow->setGeometry(QRect(5, 57, 145, 24));
        spShow->setAlignment(Qt::AlignmentFlag::AlignLeading|Qt::AlignmentFlag::AlignLeft|Qt::AlignmentFlag::AlignTop);
        spLabel = new QLabel(saveGroup);
        spLabel->setObjectName(QString::fromUtf8("spLabel"));
        spLabel->setGeometry(QRect(5, 42, 80, 15));
        saveButton = new QPushButton(saveGroup);
        saveButton->setObjectName(QString::fromUtf8("saveButton"));
        saveButton->setGeometry(QRect(10, 20, 60, 20));
        QIcon icon2(QIcon::fromTheme(QString::fromUtf8("QIcon::ThemeIcon::DocumentSave")));
        saveButton->setIcon(icon2);
        isSaveCloud = new QCheckBox(saveGroup);
        isSaveCloud->setObjectName(QString::fromUtf8("isSaveCloud"));
        isSaveCloud->setGeometry(QRect(80, 20, 80, 20));

        retranslateUi(panel);

        QMetaObject::connectSlotsByName(panel);
    } // setupUi

    void retranslateUi(QWidget *panel)
    {
        panel->setWindowTitle(QCoreApplication::translate("panel", "Form", nullptr));
        settingGroup->setTitle(QCoreApplication::translate("panel", "setting", nullptr));
        runGroup->setTitle(QCoreApplication::translate("panel", "run", nullptr));
        runButton->setText(QCoreApplication::translate("panel", "run", nullptr));
        nextButton->setText(QCoreApplication::translate("panel", "next", nullptr));
        previousButton->setText(QCoreApplication::translate("panel", "previous", nullptr));
        cloudButton->setText(QCoreApplication::translate("panel", "cloud", nullptr));
        loaderGroup->setTitle(QCoreApplication::translate("panel", "loader", nullptr));
        QMLabel->setText(QCoreApplication::translate("panel", "Q Matrix:", nullptr));
        QFileShow->setText(QCoreApplication::translate("panel", "--", nullptr));
        QFile->setText(QString());
        listMode->setText(QCoreApplication::translate("panel", "ListFile", nullptr));
        dirMode->setText(QCoreApplication::translate("panel", "ImageFile", nullptr));
        index->setText(QCoreApplication::translate("panel", "0", nullptr));
        total->setText(QCoreApplication::translate("panel", "/0", nullptr));
        AlgorithmsGroup->setTitle(QCoreApplication::translate("panel", "Algorithms", nullptr));
        SGBMButton->setText(QCoreApplication::translate("panel", "SGBM", nullptr));
        ELASButton->setText(QCoreApplication::translate("panel", "ELAS", nullptr));
        BMButton->setText(QCoreApplication::translate("panel", "BM", nullptr));
        saveGroup->setTitle(QCoreApplication::translate("panel", "save", nullptr));
        cloudPathButton->setText(QString());
        cloudPath->setText(QCoreApplication::translate("panel", "--", nullptr));
        cloudLabel->setText(QCoreApplication::translate("panel", "cloud path:", nullptr));
        savePathButton->setText(QString());
        spShow->setText(QCoreApplication::translate("panel", "--", nullptr));
        spLabel->setText(QCoreApplication::translate("panel", "save path:", nullptr));
        saveButton->setText(QCoreApplication::translate("panel", "save", nullptr));
        isSaveCloud->setText(QCoreApplication::translate("panel", "save cloud", nullptr));
    } // retranslateUi

};

namespace Ui {
    class panel: public Ui_panel {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PANEL_H
