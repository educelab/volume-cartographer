// CWindow.cpp
// Chao Du 2014 Dec
#include "CWindow.h"

#include "HBase.h"

#include <QtCore>
#include <QtGui>

#include "volumepkg.h"
#include "CVolumeViewerWithCurve.h"

#include "UDataManipulateUtils.h"
#include "structureTensorParticleSim.h"

#define _DEBUG

using namespace ChaoVis;


// Constructor
CWindow::CWindow( void ) :
    fVpkg( NULL ),
    fPathOnSliceIndex( 0 ),
    fVolumeViewerWidget( NULL ),
    fPathListWidget( NULL ),
    fPenTool( NULL ),
    fEditTool( NULL ),
    fIsInDrawingMode( false ),
    fIsInEditingMode( false )
{
    ui.setupUi( this );

    // default parameters for segmentation method
    // REVISIT - refactor me
    fSegParams.fGravityScale = 1;
    fSegParams.fThreshold = 1;
    fSegParams.fEndOffset = -1;

    // create UI widgets
    CreateWidgets();

    // create menu
    CreateActions();
    CreateMenus();

#ifdef _DEBUG
    if ( fVolumeViewerWidget == NULL ) {
        QMessageBox::information( this, tr( "WARNING" ), tr( "Widget not found" ) );
    } else {
        Open(); // REVISIT - for debug only!

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

// Create widgets
void CWindow::CreateWidgets( void )
{
    // add volume viewer
    QWidget *aTabSegment = this->findChild< QWidget * >( "tabSegment" );
    assert( aTabSegment != NULL );

    fVolumeViewerWidget = new CVolumeViewerWithCurve();

    QVBoxLayout *aWidgetLayout = new QVBoxLayout;
    aWidgetLayout->addWidget( fVolumeViewerWidget );

    aTabSegment->setLayout( aWidgetLayout );

    // pass the reference of the curve to the widget
    fVolumeViewerWidget->SetCurve( fCurve );

    // new path button
    QPushButton *aBtnNewPath = this->findChild< QPushButton * >( "btnNewPath" );
    connect( aBtnNewPath, SIGNAL( clicked() ), this, SLOT( OnNewPathClicked() ) );

    // pen tool and edit tool
    fPenTool = this->findChild< QPushButton * >( "btnPenTool" );
    fEditTool = this->findChild< QPushButton * >( "btnEditTool" );
    connect( fPenTool, SIGNAL( clicked() ), this, SLOT( TogglePenTool() ) );
    connect( fEditTool, SIGNAL( clicked() ), this, SLOT( ToggleEditTool() ) );

    // list of paths
    fPathListWidget = this->findChild< QListWidget * >( "lstPaths" );
    connect( fPathListWidget, SIGNAL( itemClicked( QListWidgetItem* ) ), this, SLOT( OnPathItemClicked( QListWidgetItem* ) ) );

    // segmentation methods
    QComboBox *aSegMethodsComboBox = this->findChild< QComboBox * >( "cmbSegMethods" );
    aSegMethodsComboBox->addItem( tr( "Structure Tensor Particle Simulation" ) );

    fEdtGravity = this->findChild< QLineEdit * >( "edtGravityVal" );
    fEdtSampleDist = this->findChild< QLineEdit * >( "edtSampleDistVal" );
    fEdtStartIndex = this->findChild< QLineEdit * >( "edtStartingSliceVal" );
    fEdtEndIndex = this->findChild< QLineEdit * >( "edtEndingSliceVal" );
    // REVISIT - consider switching to CSimpleNumEditBox
    // see QLineEdit doc to see the difference of textEdited() and textChanged()
    // doc.qt.io/qt-4.8/qlineedit.html#textEdited
    connect( fEdtGravity, SIGNAL( textEdited(QString) ), this, SLOT( OnEdtGravityValChange( QString ) ) );
    connect( fEdtSampleDist, SIGNAL( textEdited(QString) ), this, SLOT( OnEdtSampleDistValChange( QString ) ) );
    connect( fEdtStartIndex, SIGNAL( textEdited(QString) ), this, SLOT( OnEdtStartingSliceValChange( QString ) ) );
    connect( fEdtEndIndex, SIGNAL( textEdited(QString) ), this, SLOT( OnEdtEndingSliceValChange( QString ) ) );

    // start segmentation button
    QPushButton *aBtnStartSeg = this->findChild< QPushButton * >( "btnStartSeg" );
    connect( aBtnStartSeg, SIGNAL( clicked() ), this, SLOT( OnBtnStartSegClicked() ) );
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

// Update the widgets
void CWindow::UpdateView( void )
{
    if ( fVpkg == NULL ) {
        return;
    }

    // show volume package name
    this->findChild< QLabel * >( "lblVpkgName" )->setText( QString( fVpkg->getPkgName().c_str() ) );

    // show the existing paths
    for ( size_t i = 0; i < fVpkg->getSegmentations().size(); ++i ) {
        fPathListWidget->addItem( new QListWidgetItem( QString( fVpkg->getSegmentations()[ i ].c_str() ) ) );
    }

    // set widget accessibility properly based on the states: is drawing? is editing?
    // REVISIT - FILL ME HERE
    fEdtGravity->setText( QString( "%1" ).arg( fSegParams.fGravityScale ) );
    fEdtSampleDist->setText( QString( "%1" ).arg( fSegParams.fThreshold ) );
    fEdtStartIndex->setText( QString( "%1" ).arg( fPathOnSliceIndex ) );
    fEdtEndIndex->setText( QString( "%1" ).arg( fSegParams.fEndOffset ) );
}

// Do segmentation given the starting point cloud
void CWindow::DoSegmentation( void )
{
    // REVISIT - do we need to get the latest value from the widgets since we constantly get the values?
    if ( !SetUpSegParams() ) {
        QMessageBox::information( this, tr( "Info" ), tr( "Invalid parameter for segmentation" ) );
        return;
    }

    // how to create pcl::PointCloud::Ptr from a pcl::PointCloud?
    // stackoverflow.com/questions/10644429/create-a-pclpointcloudptr-from-a-pclpointcloud
    fICloud = structureTensorParticleSim( pcl::PointCloud< pcl::PointXYZRGB >::Ptr( &fPathCloud ),
                                            *fVpkg,
                                            fSegParams.fGravityScale,
                                            fSegParams.fThreshold,
                                            fSegParams.fEndOffset );
}

// Set up the parameters for doing segmentation
bool CWindow::SetUpSegParams( void )
{
    bool aIsOk;

    // gravity value
    int aNewVal = fEdtGravity->text().toInt( &aIsOk );
    if ( aIsOk ) {
        fSegParams.fGravityScale = aNewVal;
    } else {
        return false;
    }

    // sample distance
    aNewVal = fEdtSampleDist->text().toInt( &aIsOk );
    if ( aIsOk ) {
        fSegParams.fThreshold = aNewVal;
    } else {
        return false;
    }

    // starting slice index is fPathOnSliceIndex

    // ending slice index
    aNewVal = fEdtEndIndex->text().toInt( &aIsOk );
    if ( aIsOk ) {
        fSegParams.fEndOffset = aNewVal;
    } else {
        return false;
    }

    return true;
}

void CWindow::Open( void )
{
    QString aVpkgPath = QString( "" );
    aVpkgPath = QFileDialog::getExistingDirectory( this,
                                                   tr( "Open Directorry" ),
#ifdef _DEBUG
                                                   "/home/chaodu/Research/Scroll/meshEditorData/VolPkgs/",
#else
                                                   QDir::homePath(),
#endif // _DEBUG
                                                   QFileDialog::ShowDirsOnly |
                                                   QFileDialog::DontResolveSymlinks );
    if ( aVpkgPath.length() == 0 ) { // canceled
        return;
    }

    if ( !InitializeVolumePkg( aVpkgPath.toStdString() + "/" ) ) {
        printf( "cannot open the volume package at the specified location.\n" );
        return;
    }

    fVpkgPath = aVpkgPath;

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

    UpdateView(); // update the panel when volume package is loaded
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
    QMessageBox::information( this, tr( "About Volume Cartographer" ), tr( "Vis Center, University of Kentucky" ) );
}

// Create new path
void CWindow::OnNewPathClicked( void )
{
    // calculate the path and save that to aPathCloud
    std::vector< cv::Vec2f > aSamplePts;
    fCurve.GetSamplePoints( aSamplePts );

    pcl::PointXYZRGB point;
    for ( size_t i = 0; i < aSamplePts.size(); ++i ) {
        point.x = fPathOnSliceIndex;
        point.y = aSamplePts[ i ][ 0 ];
        point.z = aSamplePts[ i ][ 1 ];
        fPathCloud.push_back( point );
    }

    fVpkg->setActiveSegmentation( fVpkg->newSegmentation() );
    fVpkg->saveCloud( fPathCloud );
}

// Handle path item click event
void CWindow::OnPathItemClicked( QListWidgetItem* nItem )
{
//    QMessageBox::information( this, tr( "Info" ), nItem->text() );
    QString aPathFileName = fVpkgPath + "/" + nItem->text() + "/" + "path.pcd"; // REVISIT - naming convention
//    QMessageBox::information( this, tr( "Info" ), aPathFileName );

    // REVISIT - load proper point cloud
}

// Toggle the status of the pen tool
void CWindow::TogglePenTool( void )
{
//    QMessageBox::information( this, tr( "info" ), tr( "Pen tool status toggled" ) );
    fIsInDrawingMode = fPenTool->isChecked();
}

// Toggle the status of the edit tool
void CWindow::ToggleEditTool( void )
{
//    QMessageBox::information( this, tr( "info" ), tr( "Edit tool status toggled" ) );
    fIsInEditingMode = fEditTool->isChecked();
}

// Handle gravity value change
void CWindow::OnEdtGravityValChange( QString nText )
{
    bool aIsOk;
    int aNewVal = nText.toInt( &aIsOk );
    if ( aIsOk ) {
        fSegParams.fGravityScale = aNewVal;
    }
}

// Handle sample distance value change
void CWindow::OnEdtSampleDistValChange( QString nText )
{
    // REVISIT - the widget should be disabled and the change ignored for now
    bool aIsOk;
    int aNewVal = nText.toInt( &aIsOk );
    if ( aIsOk ) {
        fSegParams.fThreshold = aNewVal; // REVISIT - is this the correct member variable to assign to?
    }
}

// Handle starting slice value change
void CWindow::OnEdtStartingSliceValChange( QString nText )
{
    // REVISIT - FILL ME HERE
}

// Handle ending slice value change
void CWindow::OnEdtEndingSliceValChange( QString nText )
{
    // REVISIT - FILL ME HERE
}

// Handle start segmentation
void CWindow::OnBtnStartSegClicked( void )
{
    DoSegmentation();
}
