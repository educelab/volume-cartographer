// main.cpp
// Chao Du 2014 Dec

#include <QDesktopWidget>
#include <qapplication.h>
#include "CWindow.h"

using namespace ChaoVis;

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    QRect rec =
        QApplication::desktop()->screenGeometry();  // Instantiates a QRec
                                                    // "Rectangle" Object and
                                                    // gives it the dimensions
                                                    // of the screen display
    CWindow aWin(rec);
    aWin.show();
    return app.exec();
}
