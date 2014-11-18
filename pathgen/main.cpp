// main.cpp
// Chao Du Nov 2014
// qt-project.org/doc/qt-4.8/widgest-imageviewer-main-cpp.html

#include <QApplication>

#include "CQtImageViewer.h"

#include <string>

int main( int argc, char *argv[] )
{
    QApplication app( argc, argv );
	CQtImageViewer *aImageViewer = NULL;
	if ( argc > 1 ) {	// with extra parameters
		if ( argc != 3 || strncmp( argv[ 1 ], "--help", 6 ) == 0 ) {
			printf( "Usage: vc_pathgen pathToVolPkg sliceIndex\n" );
			exit( -1 );
		}
		aImageViewer = new CQtImageViewer( std::string( argv[ 1 ] ), atoi( argv[ 2 ] ) );
	} else {
		aImageViewer = new CQtImageViewer();
	}

#if defined( Q_OS_SYMBIAN )
	aImageViewer->showMaximized();
#else
	aImageViewer->show();
#endif

	int aResult = app.exec();

	if ( aImageViewer != NULL ) {
		delete aImageViewer;
		aImageViewer = NULL;
	}
	return aResult;
}
