// main.cpp
// Chao Du 2014 Dec

#include <QDesktopWidget>
#include <qapplication.h>
#include "CWindow.hpp"

using namespace ChaoVis;

auto main(int argc, char* argv[]) -> int
{
    QApplication app(argc, argv);
    QApplication::setApplicationName("VC");
    QApplication::setOrganizationName("EduceLab, University of Kentucky");
    QApplication::setOrganizationDomain("cs.uky.edu/dri");

    qRegisterMetaType<CWindow::Segmenter>("Segmenter");
    qRegisterMetaType<CWindow::Segmenter::PointSet>(
        "CWindow::Segmenter::PointSet");

    QRect rec = QGuiApplication::primaryScreen()->geometry();
    CWindow aWin(rec);
    aWin.show();
    return QApplication::exec();
}
