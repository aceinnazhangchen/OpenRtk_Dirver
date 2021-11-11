#include "OpenRtk_Dirver.h"
#include <QtWidgets/QApplication>

#define PROGRAM "OpenRTK Driver"
#define VERSION "v1.7.6"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    OpenRtk_Dirver w;
	w.setWindowTitle(PROGRAM + QString("-") + VERSION);
    w.show();
    return a.exec();
}