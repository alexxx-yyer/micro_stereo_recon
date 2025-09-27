#pragma once
#include <QWidget>
#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>
#include <QFileDialog>
#include <QPlainTextEdit>

class LoadWidget : public QWidget
{
    Q_OBJECT
public:
    LoadWidget(QWidget *parent = nullptr) : QWidget(parent) {}

    virtual ~LoadWidget() {};
    virtual void show() = 0;

public:

signals:
    void selectListFile(QString);
    void leftFile(QString);
    void rightFile(QString);
    void leftDir(QString);
    void rightDir(QString);
};

class LoadList : public LoadWidget
{
    Q_OBJECT
public:
    QPushButton *fileButton;
    QLabel *label;
    QLabel *path;

    ~LoadList() {
        delete label;
        delete path;
        delete fileButton;
    }

    LoadList(QWidget *loadlist) : LoadWidget(loadlist) {
        fileButton = new QPushButton(loadlist);
        fileButton->setObjectName("fileButton");
        fileButton->setGeometry(QRect(140, 50, 25, 25));
        fileButton->setMaximumSize(QSize(25, 25));
        QIcon icon(QIcon::fromTheme(QIcon::ThemeIcon::DocumentNew));
        fileButton->setIcon(icon);
        label = new QLabel(loadlist);
        label->setObjectName("label");
        label->setGeometry(QRect(10, 50, 30, 25));
        label->setMaximumSize(QSize(30, 25));
        label->setAlignment(Qt::AlignmentFlag::AlignLeading|Qt::AlignmentFlag::AlignLeft|Qt::AlignmentFlag::AlignVCenter);
        path = new QLabel(loadlist);
        path->setObjectName("path");
        path->setGeometry(QRect(10, 80, 151, 60));
        path->setAlignment(Qt::AlignmentFlag::AlignLeading|Qt::AlignmentFlag::AlignLeft|Qt::AlignmentFlag::AlignTop);
        path->setWordWrap(true);
        path->setTextInteractionFlags(Qt::TextInteractionFlag::TextBrowserInteraction);

        retranslateUi(loadlist);

        connect(fileButton, &QPushButton::clicked, this, &LoadList::sendClicked);
    }

    void show() {
        fileButton->show();
        label->show();
        path->show();
    }

    void retranslateUi(QWidget *loadlist)
    {
        loadlist->setWindowTitle(QCoreApplication::translate("loadlist", "Form", nullptr));
        fileButton->setText(QString());
        label->setText(QCoreApplication::translate("loadlist", "file:", nullptr));
        path->setText(QCoreApplication::translate("loadlist", "--", nullptr));
    } // retranslateUi

private slots:
    void sendClicked() {
        QString file = QFileDialog::getOpenFileName(this, "Select list file", "", "(*.xml *.yml *.yaml)");
        path->setText(file);
        emit selectListFile(file);
    }
};

class LoadFile : public LoadWidget
{
    Q_OBJECT
public:
    QLabel *label;
    QLabel *label_3;
    QLabel *lpath;
    QLabel *rpath;
    QPushButton *lfileButton;
    QPushButton *rfileButton;
    QPushButton *ldirButton;
    QPushButton *rdirButton;

    ~LoadFile() {
        delete label;
        delete lpath;
        delete label_3;
        delete rpath;
        delete lfileButton;
        delete rfileButton;
        delete ldirButton;
        delete rdirButton;
    }

    LoadFile(QWidget *loadfile) : LoadWidget(loadfile) {
        label = new QLabel(loadfile);
        label->setObjectName("label");
        label->setGeometry(QRect(10, 50, 35, 20));
        label->setMaximumSize(QSize(35, 20));
        label_3 = new QLabel(loadfile);
        label_3->setObjectName("label_3");
        label_3->setGeometry(QRect(10, 110, 35, 20));
        label_3->setMaximumSize(QSize(35, 20));
        lpath = new QLabel(loadfile);
        lpath->setObjectName("lpath");
        lpath->setGeometry(QRect(10, 70, 160, 40));
        QFont font;
        font.setPointSize(7);
        lpath->setFont(font);
        lpath->setAlignment(Qt::AlignmentFlag::AlignLeading|Qt::AlignmentFlag::AlignLeft|Qt::AlignmentFlag::AlignTop);
        lpath->setWordWrap(true);
        lpath->setTextInteractionFlags(Qt::TextInteractionFlag::TextBrowserInteraction);
        rpath = new QLabel(loadfile);
        rpath->setObjectName("rpath");
        rpath->setGeometry(QRect(10, 130, 160, 40));
        rpath->setFont(font);
        rpath->setAlignment(Qt::AlignmentFlag::AlignLeading|Qt::AlignmentFlag::AlignLeft|Qt::AlignmentFlag::AlignTop);
        rpath->setWordWrap(true);
        rpath->setTextInteractionFlags(Qt::TextInteractionFlag::TextBrowserInteraction);
        lfileButton = new QPushButton(loadfile);
        lfileButton->setObjectName("lfileButton");
        lfileButton->setGeometry(QRect(130, 45, 25, 25));
        lfileButton->setMaximumSize(QSize(25, 25));
        QIcon icon(QIcon::fromTheme(QIcon::ThemeIcon::DocumentNew));
        lfileButton->setIcon(icon);
        rfileButton = new QPushButton(loadfile);
        rfileButton->setObjectName("rfileButton");
        rfileButton->setGeometry(QRect(130, 105, 25, 24));
        rfileButton->setMaximumSize(QSize(25, 25));
        rfileButton->setIcon(icon);
        ldirButton = new QPushButton(loadfile);
        ldirButton->setObjectName("ldirButton");
        ldirButton->setGeometry(QRect(155, 45, 25, 25));
        ldirButton->setMaximumSize(QSize(25, 25));
        QIcon icon1(QIcon::fromTheme(QIcon::ThemeIcon::FolderOpen));
        ldirButton->setIcon(icon1);
        rdirButton = new QPushButton(loadfile);
        rdirButton->setObjectName("rdirButton");
        rdirButton->setGeometry(QRect(155, 105, 25, 25));
        rdirButton->setMaximumSize(QSize(25, 25));
        rdirButton->setIcon(icon1);

        retranslateUi(loadfile);

        connect(lfileButton, &QPushButton::clicked, this, &LoadFile::leftFileClicked);
        connect(rfileButton, &QPushButton::clicked, this, &LoadFile::rightFileClicked);
        connect(ldirButton, &QPushButton::clicked, this, &LoadFile::leftDirClicked);
        connect(rdirButton, &QPushButton::clicked, this, &LoadFile::rightDirClicked);
    }

    void show() {
        label->show();
        label_3->show();
        lpath->show();
        rpath->show();
        lfileButton->show();
        rfileButton->show();
        ldirButton->show();
        rdirButton->show();
    }

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

public slots:
    void leftFileClicked() {
        QString path = QFileDialog::getOpenFileName(this, "Select an image", "", "(*.png *.jpg)");
        lpath->setText(path);
        emit leftFile(path);
    }

    void rightFileClicked() {
        QString path = QFileDialog::getOpenFileName(this, "Select an image", "", "(*.png *.jpg)");
        rpath->setText(path);
        emit rightFile(path);
    }

    void leftDirClicked() {
        QString path = QFileDialog::getExistingDirectory(this, "Select directory");
        lpath->setText(path);
        emit leftDir(path);
    }

    void rightDirClicked() {
        QString path = QFileDialog::getExistingDirectory(this, "Select directory");
        rpath->setText(path);
        emit rightDir(path);
    }
};