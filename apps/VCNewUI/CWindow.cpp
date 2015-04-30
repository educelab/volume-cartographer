// CWindow.cpp
// Chao Du 2014 Dec
#include "CWindow.h"

#include "HBase.h"

#include <QtCore>
#include <QtGui>

#include "volumepkg/volumepkg.h"
#include "CVolumeViewerWithCurve.h"

#include "UDataManipulateUtils.h"

#define _DEBUG

using namespace ChaoVis;


// Constructor
CWindow::CWindow( void ) :
    fVpkg( NULL ),
    fPathOnSliceIndex( 0 ),
    fVolumeViewerWidget( NULL )
{
    ui.setupUi( this );

    QWidget *aTabSegment = this->findChild< QWidget * >( "tabSegment" );
    assert( aTabSegment != NULL );

    fVolumeViewerWidget = new CVolumeViewerWithCurve();

	QVBoxLayout *aWidgetLayout = new QVBoxLayout;
    aWidgetLayout->addWidget( fVolumeViewerWidget );

    aTabSegment->setLayout( aWidgetLayout );

    fVolumeViewerWidget->SetCurve( fCurve );

    QPushButton *aBtnNewPath = this->findChild< QPushButton * >( "btnNewPath" );
    connect( aBtnNewPath, SIGNAL( clicked() ), this, SLOT( OnNewPathClicked() ) );

    CreateActions();
    CreateMenus();

#ifdef _DEBUG
    if ( fVolumeViewerWidget == NULL ) {
        QMessageBox::information( this, tr( "WARNING" ), tr( "Widget not found" ) );
    } else {
        Open(); // REVISIT - for debug only!
        this->findChild< QLabel * >( "lblVpkgName" )->setText( QString( fVpkg->getPkgName().c_str() ) );

        cv::Mat aImgMat;
        fVpkg->getSliceAtIndex( fPathOnSliceIndex ).copyTo( aImgMat );
        aImgMat.convertTo( aImgMat, CV_8UC3, 1.0 / 256.0 );
        cvtColor( aImgMat, aImgMat, CV_GRAY2BGR );

        QImage aImgQImage;
        aImgQImage = Mat2QImage( aImgMat );
        fVolumeViewerWidget->SetImage( aImgQImage );

        update();
    }
#endif // _DEBUG
}

// Destructor
CWindow::~CWindow( void )
{
}

// Handle mouse press event
void CWindow::mousePressEvent( QMouseEvent *nEvent )
{
}

// Handle key press event
void CWindow::keyPressEvent( QKeyEvent *event )
{
	if ( event->key() == Qt::Key_Escape ) {
		// REVISIT - should prompt warning before exit
		close();
	} else {
		// REVISIT - dispatch key press event
	}
}

// Create menus
void CWindow::CreateMenus( void )
{
    fFileMenu = new QMenu( tr( "&File" ), this );
    fFileMenu->addAction( fOpenVolAct );
    fFileMenu->addSeparator();
    fFileMenu->addAction( fExitAct );

    fEditMenu = new QMenu( tr( "&Edit" ), this );
//	fEditMenu->addAction( fGetIntersectionAct );

    fHelpMenu = new QMenu( tr( "&Help" ), this );
    fHelpMenu->addAction( fAboutAct );

    menuBar()->addMenu( fFileMenu );
    menuBar()->addMenu( fEditMenu );
    menuBar()->addMenu( fHelpMenu );
}

// Create actions
void CWindow::CreateActions( void )
{
    fOpenVolAct = new QAction( tr( "&Open volume..." ), this );
    connect( fOpenVolAct, SIGNAL( triggered() ), this, SLOT( Open() ) );
    fExitAct = new QAction( tr( "E&xit..." ), this );
    connect( fExitAct, SIGNAL( triggered() ), this, SLOT( Close() ) );
    fAboutAct = new QAction( tr( "&About..." ), this );
    connect( fAboutAct, SIGNAL( triggered() ), this, SLOT( About() ) );
}

bool CWindow::InitializeVolumePkg( const std::string &nVpkgPath )
{
    deleteNULL( fVpkg );
    fVpkg = new VolumePkg( nVpkgPath );

    if ( fVpkg == NULL ) {
        printf( "ERROR: cannot open volume package %s\n", nVpkgPath.c_str() );
        return false;
    }
    return true;
}

// Do segmentation given the starting point cloud
void CWindow::DoSegmentation( void )
{
    // REVISIT - FILL ME HERE
}

void CWindow::Open( void )
{
    QString aVpkgPath = QString( "" );
    aVpkgPath = QFileDialog::getExistingDirectory( this,
                                                   tr( "Open Directorry" ),
                                                   QDir::homePath(),
                                                   QFileDialog::ShowDirsOnly |
                                                   QFileDialog::DontResolveSymlinks );
    if ( aVpkgPath.length() == 0 ) { // canceled
        return;
    }

    if ( !InitializeVolumePkg( aVpkgPath.toStdString() + "/" ) ) {
        printf( "cannot open the volume package at the specified location.\n" );
        return;
    }

    // get the slice index
    bool aIsIndexOk = false;
    fPathOnSliceIndex = QInputDialog::getInt( this,
                                              tr( "Input" ),
                                              tr( "Please enter the index of the slice you want to draw path on: " ),
                                              2,                    // default value
                                              2,                    // minimum value
                                              fVpkg->getNumberOfSlices() - 3,   // maximum vlaue
                                              1,                    // step
                                              &aIsIndexOk );
    if ( !aIsIndexOk ) {
        printf( "WARNING: nothing was loaded because no slice index was specified.\n" );
        return;
    }
}

// Close application
void CWindow::Close( void )
{
    close();
}

// Pop up about dialog
void CWindow::About( void )
{
    // REVISIT - FILL ME HERE
}

// Create new path
void CWindow::OnNewPathClicked( void )
{
    pcl::PointCloud< pcl::PointXYZRGB > aPathCloud;

    // calculate the path and save that to aPathCloud
    std::vector< cv::Vec2f > aSamplePts;
    fCurve.GetSamplePoints( aSamplePts );

    pcl::PointXYZRGB point;
    for ( size_t i = 0; i < aSamplePts.size(); ++i ) {
        point.x = fPathOnSliceIndex;
        point.y = aSamplePts[ i ][ 0 ];
        point.z = aSamplePts[ i ][ 1 ];
        aPathCloud.push_back( point );
    }

    fVpkg->setActiveSegmentation( fVpkg->newSegmentation() );
    fVpkg->saveCloud( aPathCloud );
}
