/********************************************************************************
** Form generated from reading UI file 'loadlist.ui'
**
** Created by: Qt User Interface Compiler version 5.15.13
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOADLIST_H
#define UI_LOADLIST_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_loadlist
{
public:
    QPushButton *fileButton;
    QLabel *label;
    QLabel *path;

    void setupUi(QWidget *loadlist)
    {
        if (loadlist->objectName().isEmpty())
            loadlist->setObjectName(QString::fromUtf8("loadlist"));
        loadlist->resize(185, 270);
        fileButton = new QPushButton(loadlist);
        fileButton->setObjectName(QString::fromUtf8("fileButton"));
        fileButton->setGeometry(QRect(140, 50, 25, 25));
        fileButton->setMaximumSize(QSize(25, 25));
        QIcon icon(QIcon::fromTheme(QString::fromUtf8("QIcon::ThemeIcon::DocumentNew")));
        fileButton->setIcon(icon);
        label = new QLabel(loadlist);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 50, 30, 25));
        label->setMaximumSize(QSize(30, 25));
        label->setAlignment(Qt::AlignmentFlag::AlignLeading|Qt::AlignmentFlag::AlignLeft|Qt::AlignmentFlag::AlignVCenter);
        path = new QLabel(loadlist);
        path->setObjectName(QString::fromUtf8("path"));
        path->setGeometry(QRect(10, 80, 151, 60));
        path->setAlignment(Qt::AlignmentFlag::AlignLeading|Qt::AlignmentFlag::AlignLeft|Qt::AlignmentFlag::AlignTop);
        path->setWordWrap(true);
        path->setTextInteractionFlags(Qt::TextInteractionFlag::TextBrowserInteraction);

        retranslateUi(loadlist);

        QMetaObject::connectSlotsByName(loadlist);
    } // setupUi

    void retranslateUi(QWidget *loadlist)
    {
        loadlist->setWindowTitle(QCoreApplication::translate("loadlist", "Form", nullptr));
        fileButton->setText(QString());
        label->setText(QCoreApplication::translate("loadlist", "file:", nullptr));
        path->setText(QCoreApplication::translate("loadlist", "--", nullptr));
    } // retranslateUi

};

namespace Ui {
    class loadlist: public Ui_loadlist {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOADLIST_H
