// main.cpp
// Chao Du 2014 Dec

#include <QScreen>
#include <qapplication.h>

#include "CWindow.hpp"
#include "vc/core/Version.hpp"

using namespace ChaoVis;

namespace vc = volcart;

auto main(int argc, char* argv[]) -> int
{
    QApplication app(argc, argv);
    QApplication::setOrganizationName("EduceLab");
    QApplication::setApplicationName("VC");
    QApplication::setApplicationVersion(
        QString::fromStdString(vc::ProjectInfo::VersionString()));

    qRegisterMetaType<CWindow::Segmenter>("Segmenter");
    qRegisterMetaType<CWindow::Segmenter::PointSet>(
        "CWindow::Segmenter::PointSet");

    CWindow aWin;
    aWin.show();
    return QApplication::exec();
}
