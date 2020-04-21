// main.cpp
// Chao Du 2014 Dec

#include <QDesktopWidget>
#include <qapplication.h>
#include "CWindow.hpp"

using namespace ChaoVis;

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    qRegisterMetaType<CWindow::Segmenter>("Segmenter");
    qRegisterMetaType<CWindow::Segmenter::PointSet>(
        "CWindow::Segmenter::PointSet");

    QRect rec = QApplication::desktop()->screenGeometry();
    CWindow aWin(rec);
    aWin.show();
    return app.exec();
}
