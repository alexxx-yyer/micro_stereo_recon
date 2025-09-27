#include <QApplication>
#include "window.h"

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    QApplication::setStyle("windowsvista");
    window w;
    w.show();
    return a.exec();
}