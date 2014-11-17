// main.cpp
// Chao Du Nov 2014
// qt-project.org/doc/qt-4.8/widgest-imageviewer-main-cpp.html

#include <QApplication>

#include "CQtImageViewer.h"

int main( int argc, char *argv[] )
{
    QApplication app( argc, argv );
	CQtImageViewer aImageViewer;

#if defined( Q_OS_SYMBIAN )
	aImageViewer.showMaximized();
#else
	aImageViewer.show();
#endif

	return app.exec();
}
