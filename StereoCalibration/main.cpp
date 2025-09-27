#include <QtWidgets/QApplication>
#include <QtWidgets/qstyle.h>
#include <QtWidgets/qstylefactory.h>
#include "panel.h"
#include <iostream>

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    QApplication::setStyle("windowsvista");
    panel w;
    w.show();
    return a.exec();
}