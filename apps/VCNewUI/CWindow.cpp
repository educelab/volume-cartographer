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
        OpenVolume(); // REVISIT - for debug only!

        OpenSlice();

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
    fVolumeViewerWidget->SetSplineCurve( fSplineCurve );
    fVolumeViewerWidget->SetIntersectionCurve( fIntersectionCurve );

    connect( fVolumeViewerWidget, SIGNAL( SendSignalOnNextClicked() ), this, SLOT( OnLoadNextSlice() ) );
    connect( fVolumeViewerWidget, SIGNAL( SendSignalOnPrevClicked() ), this, SLOT( OnLoadPrevSlice() ) );
    connect( fVolumeViewerWidget, SIGNAL( SendSignalOnLoadAnyImage(int) ), this, SLOT( OnLoadAnySlice(int) ) );

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
    connect( fOpenVolAct, SIGNAL( triggered() ), this, SLOT( OpenVolume() ) );
    fExitAct = new QAction( tr( "E&xit..." ), this );
    connect( fExitAct, SIGNAL( triggered() ), this, SLOT( Close() ) );
    fAboutAct = new QAction( tr( "&About..." ), this );
    connect( fAboutAct, SIGNAL( triggered() ), this, SLOT( About() ) );
    fSavePointCloudAct = new QAction( tr( "&Save point cloud..." ), this );
    connect( fSavePointCloudAct, SIGNAL( triggered() ), this, SLOT( SavePointCloud() ) );
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
    fEdtEndIndex->setText( QString( "%1" ).arg( fSegParams.fEndOffset + fPathOnSliceIndex ) ); // offset + starting index

    // REVISIT - these two states should be mutually exclusive, we guarantee this when we toggle the button, BUGGY!
    if ( !fIsInDrawingMode && !fIsInEditingMode ) {
        fVolumeViewerWidget->SetViewState( CVolumeViewerWithCurve::EViewState::ViewStateIdle );
    }
    if ( fIsInDrawingMode ) {
        fVolumeViewerWidget->SetViewState( CVolumeViewerWithCurve::EViewState::ViewStateDraw );
    }
    if ( fIsInEditingMode ) {
        fVolumeViewerWidget->SetViewState( CVolumeViewerWithCurve::EViewState::ViewStateEdit );
    }
}

// Do segmentation given the starting point cloud
void CWindow::DoSegmentation( void )
{
    // REVISIT - do we need to get the latest value from the widgets since we constantly get the values?
    if ( !SetUpSegParams() ) {
        QMessageBox::information( this, tr( "Info" ), tr( "Invalid parameter for segmentation" ) );
        return;
    }


    // REVISIT - we need to notify the user somewhere that segmentation can't be done when the selected path has -1(s)
    //           this can only happen in editing mode but not in drawing new path mode


    // 1) keep the immutable/upper part of the point cloud
    // see apps/segment.cpp for reference
    fUpperPart.clear();
    int aTotalNumOfImmutablePts = fMasterCloud.width * ( fPathOnSliceIndex - VOLPKG_SLICE_MIN_INDEX );
    for ( int i = 0; i < aTotalNumOfImmutablePts; ++i ) {
        fUpperPart.push_back( fMasterCloud.points[ i ] );
    }
    // resize so the parts can be concatenated
    fUpperPart.width = fMasterCloud.width;
    fUpperPart.height = fUpperPart.points.size() / fUpperPart.width;
    fUpperPart.points.resize( fUpperPart.width * fUpperPart.height );

    // 2) do segmentation from the starting slice
    // how to create pcl::PointCloud::Ptr from a pcl::PointCloud?
    // stackoverflow.com/questions/10644429/create-a-pclpointcloudptr-from-a-pclpointcloud
    fLowerPart = structureTensorParticleSim( pcl::PointCloud< pcl::PointXYZRGB >::Ptr( &fPathCloud ),
                                             *fVpkg,
                                             fSegParams.fGravityScale,
                                             fSegParams.fThreshold,
                                             fSegParams.fEndOffset );

    // 3) concatenate the two parts to form the complete point cloud
    fMasterCloud = fUpperPart + fLowerPart;
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
    if ( aIsOk && aNewVal > fPathOnSliceIndex ) {
        fSegParams.fEndOffset = aNewVal - fPathOnSliceIndex; // difference between the starting slice and ending slice
    } else {
        return false;
    }

    return true;
}

// Get the curves for all the slices
void CWindow::SetUpCurves( void )
{
    if ( fVpkg == NULL || fMasterCloud.size() == 0 ) {
        QMessageBox::information( this, tr( "Info" ), tr( "Either volume package or point cloud is not loaded" ) );
        return;
    }

    // assign rows of particles to the curves
    for ( size_t i = 0; i < fMasterCloud.height; ++i ) {
        CXCurve aCurve;
        aCurve.SetSliceIndex( i + VOLPKG_SLICE_MIN_INDEX );
        for ( size_t j = 0; j < fMasterCloud.width; ++i ) {
            aCurve.InsertPoint( Vec2< float >( fMasterCloud.points[ 0 ].y, fMasterCloud.points[ 0 ].z ) );
        }
        fIntersections.push_back( aCurve );
    }
}

// Open slice
void CWindow::OpenSlice( void )
{
    cv::Mat aImgMat;
    fVpkg->getSliceAtIndex( fPathOnSliceIndex ).copyTo( aImgMat );
    aImgMat.convertTo( aImgMat, CV_8UC3, 1.0 / 256.0 );
    cvtColor( aImgMat, aImgMat, CV_GRAY2BGR );

    QImage aImgQImage;
    aImgQImage = Mat2QImage( aImgMat );
    fVolumeViewerWidget->SetImage( aImgQImage );
}

// Open volume package
void CWindow::OpenVolume( void )
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

// Save point cloud to path directory
void CWindow::SavePointCloud( void )
{
    fVpkg->saveCloud( fMasterCloud );
    // REVISIT - do we need to save mesh?
    // fVpkg->saveMesh( fMasterCloud );
}

// Create new path
void CWindow::OnNewPathClicked( void )
{
    // calculate the path and save that to aPathCloud
    std::vector< cv::Vec2f > aSamplePts;
    fSplineCurve.GetSamplePoints( aSamplePts );

    pcl::PointXYZRGB point;
    for ( size_t i = 0; i < aSamplePts.size(); ++i ) {
        point.x = fPathOnSliceIndex;
        point.y = aSamplePts[ i ][ 0 ];
        point.z = aSamplePts[ i ][ 1 ];
        fPathCloud.push_back( point );
    }

    fVpkg->setActiveSegmentation( fVpkg->newSegmentation() );
}

// Handle path item click event
void CWindow::OnPathItemClicked( QListWidgetItem* nItem )
{
    QString aPathFileName = fVpkgPath + "/" + nItem->text() + "/" + "cloud.pcd"; // REVISIT - naming convention

    // load proper point cloud
    pcl::io::loadPCDFile< pcl::PointXYZRGB >( aPathFileName.toStdString(), fMasterCloud );
}

// Toggle the status of the pen tool
void CWindow::TogglePenTool( void )
{
//    QMessageBox::information( this, tr( "info" ), tr( "Pen tool status toggled" ) );
    fIsInDrawingMode = fPenTool->isChecked();

    UpdateView();
}

// Toggle the status of the edit tool
void CWindow::ToggleEditTool( void )
{
//    QMessageBox::information( this, tr( "info" ), tr( "Edit tool status toggled" ) );
    fIsInEditingMode = fEditTool->isChecked();

    UpdateView();
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
    // REVISIT - should be equivalent to "set current slice", the same as navigation through slices
}

// Handle ending slice value change
void CWindow::OnEdtEndingSliceValChange( QString nText )
{
    // REVISIT - FILL ME HERE
    // REVISIT - NOTE - this is not fEndOffset for segmentation parameter
}

// Handle start segmentation
void CWindow::OnBtnStartSegClicked( void )
{
    DoSegmentation();
}

// Handle loading any slice
void CWindow::OnLoadAnySlice( int nSliceIndex )
{
    fPathOnSliceIndex = nSliceIndex;
    OpenSlice();
    update();
}

// Handle loading the next slice
void CWindow::OnLoadNextSlice( void )
{
    fPathOnSliceIndex++;
    OpenSlice();
    update();
}

// Handle loading the previous slice
void CWindow::OnLoadPrevSlice( void )
{
    fPathOnSliceIndex--;
    OpenSlice();
    update();
}
