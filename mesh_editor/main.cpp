// main.cpp
// Chao Du 2014 Dec
#ifdef _WINDOWS
#include <qApplication>
#else
#include <qt4/Qt/qapplication.h>
#endif // _WINDOWS
#include <QDesktopWidget>
#include "CWindow.h"

using namespace ChaoVis;


int main( int argc, char *argv[] )
{
    QApplication app( argc, argv );
    CWindow aWin;
    aWin.resize( QSize( 640, 480 ) );

    aWin.show();
    
    return app.exec();
}
