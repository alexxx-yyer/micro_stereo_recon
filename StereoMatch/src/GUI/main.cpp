#include <QApplication>
#include "panel.h"

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    panel wd;
    wd.setWindowTitle("panel");
    wd.show();
    return a.exec();
}