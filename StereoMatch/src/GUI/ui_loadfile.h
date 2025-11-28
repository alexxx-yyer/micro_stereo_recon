/********************************************************************************
** Form generated from reading UI file 'loadfile.ui'
**
** Created by: Qt User Interface Compiler version 5.15.13
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOADFILE_H
#define UI_LOADFILE_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_loadfile
{
public:
    QLabel *label;
    QLabel *label_3;
    QLabel *lpath;
    QLabel *rpath;
    QPushButton *lfileButton;
    QPushButton *rfileButton;
    QPushButton *ldirButton;
    QPushButton *rdirButton;

    void setupUi(QWidget *loadfile)
    {
        if (loadfile->objectName().isEmpty())
            loadfile->setObjectName(QString::fromUtf8("loadfile"));
        loadfile->resize(185, 270);
        label = new QLabel(loadfile);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 50, 35, 20));
        label->setMaximumSize(QSize(35, 20));
        label_3 = new QLabel(loadfile);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 110, 35, 20));
        label_3->setMaximumSize(QSize(35, 20));
        lpath = new QLabel(loadfile);
        lpath->setObjectName(QString::fromUtf8("lpath"));
        lpath->setGeometry(QRect(10, 70, 160, 40));
        QFont font;
        font.setPointSize(7);
        lpath->setFont(font);
        lpath->setAlignment(Qt::AlignmentFlag::AlignLeading|Qt::AlignmentFlag::AlignLeft|Qt::AlignmentFlag::AlignTop);
        lpath->setWordWrap(true);
        lpath->setTextInteractionFlags(Qt::TextInteractionFlag::TextBrowserInteraction);
        rpath = new QLabel(loadfile);
        rpath->setObjectName(QString::fromUtf8("rpath"));
        rpath->setGeometry(QRect(10, 130, 160, 40));
        rpath->setFont(font);
        rpath->setAlignment(Qt::AlignmentFlag::AlignLeading|Qt::AlignmentFlag::AlignLeft|Qt::AlignmentFlag::AlignTop);
        rpath->setWordWrap(true);
        rpath->setTextInteractionFlags(Qt::TextInteractionFlag::TextBrowserInteraction);
        lfileButton = new QPushButton(loadfile);
        lfileButton->setObjectName(QString::fromUtf8("lfileButton"));
        lfileButton->setGeometry(QRect(130, 45, 25, 25));
        lfileButton->setMaximumSize(QSize(25, 25));
        QIcon icon(QIcon::fromTheme(QString::fromUtf8("QIcon::ThemeIcon::DocumentNew")));
        lfileButton->setIcon(icon);
        rfileButton = new QPushButton(loadfile);
        rfileButton->setObjectName(QString::fromUtf8("rfileButton"));
        rfileButton->setGeometry(QRect(130, 105, 25, 24));
        rfileButton->setMaximumSize(QSize(25, 25));
        rfileButton->setIcon(icon);
        ldirButton = new QPushButton(loadfile);
        ldirButton->setObjectName(QString::fromUtf8("ldirButton"));
        ldirButton->setGeometry(QRect(155, 45, 25, 25));
        ldirButton->setMaximumSize(QSize(25, 25));
        QIcon icon1(QIcon::fromTheme(QString::fromUtf8("QIcon::ThemeIcon::FolderOpen")));
        ldirButton->setIcon(icon1);
        rdirButton = new QPushButton(loadfile);
        rdirButton->setObjectName(QString::fromUtf8("rdirButton"));
        rdirButton->setGeometry(QRect(155, 105, 25, 25));
        rdirButton->setMaximumSize(QSize(25, 25));
        rdirButton->setIcon(icon1);

        retranslateUi(loadfile);

        QMetaObject::connectSlotsByName(loadfile);
    } // setupUi

    void retranslateUi(QWidget *loadfile)
    {
        loadfile->setWindowTitle(QCoreApplication::translate("loadfile", "Form", nullptr));
        label->setText(QCoreApplication::translate("loadfile", "left:", nullptr));
        label_3->setText(QCoreApplication::translate("loadfile", "right:", nullptr));
        lpath->setText(QCoreApplication::translate("loadfile", "--", nullptr));
        rpath->setText(QCoreApplication::translate("loadfile", "--", nullptr));
        lfileButton->setText(QString());
        rfileButton->setText(QString());
        ldirButton->setText(QString());
        rdirButton->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class loadfile: public Ui_loadfile {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOADFILE_H
