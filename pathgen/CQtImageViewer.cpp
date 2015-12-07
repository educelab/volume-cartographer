// CQtImageViewer.cpp
// Chao Du Nov 2014

#include "CQtImageViewer.h"

#include <QtCore>
#include <QtWidgets>

#ifndef QT_NO_PRINTER
#include <QPrintDialog>
#include <QPainter>
#endif // QT_NO_PRINTER

#include "timeUtils.h"


// initialize volume package
bool CQtImageViewer::InitializeVolumePkg( const std::string &nVpkgPath )
{
    if ( fVpkg != NULL ) {
        delete fVpkg;
        fVpkg = NULL;
    }
    // REVISIT - add exception handler to VolumePkg
    fVpkg = new VolumePkg( nVpkgPath );

    if ( fVpkg == NULL ) {
        return false;
    }

    if ( fVpkg->getVersion() < 2.0) {
        std::cerr << "ERROR: Volume package is version " << fVpkg->getVersion() << " but this program requires a version >= 2.0." << std::endl;
        return false;
    }

    fVpkgPath = QString::fromStdString( nVpkgPath );
    fVpkgName = fVpkg->getPkgName();
    return true;
}

// show slice
void CQtImageViewer::OpenSlice( int nPathOnSliceIndex )
{
    // get image as cv::Mat
    fVpkg->getSliceData( fPathOnSliceIndex ).copyTo( fImgMat );

    // convert 16-bit Mat to 8-bit 3-channel Mat
    // REVISIT - !!! for display purpose, we need to convert 16-bit volume slice to 8-bit image here
    //           this should be the same as the conversion in texturing program
    // TODO refactor the conversion-for-display functions and reuse here
    fImgMat.convertTo( fImgMat, CV_8U, 1.0/256.0 );
    cvtColor( fImgMat, fImgMat, CV_GRAY2BGR );
    fImgMat.copyTo( fImgMatCache ); // save a copy of previous state

    // convert cv::Mat to Qt::QImage
    fImgQImage = Mat2QImage( fImgMat );
}

// initialize view
void CQtImageViewer::InitializeView( void )
{
    fImageLabel->setPixmap( QPixmap::fromImage( fImgQImage ) );
    fScaleFactor = 1.0;

    fPrintAct->setEnabled( true );
    fFitToWindowAct->setEnabled( true );
    UpdateActions();

    if ( !fFitToWindowAct->isChecked() ) {
        fImageLabel->adjustSize();
    }

    // enable mouse tracking
    setMouseTracking( true );
    fScrollArea->setMouseTracking( true );
    fImageLabel->setMouseTracking( true );
}

// update the view
void CQtImageViewer::UpdateView( void )
{
    fImgMatCache.copyTo( fImgMat );

    for ( size_t i = 0; i < fPath.size(); ++i ) {
        DrawBezier( fPath[ i ].start,
                    fPath[ i ].middle,
                    fPath[ i ].end,
                    fImgMat );
    }

    fImgMat.copyTo( fImgMatCache );

    if ( fTupleIndex == 2 ) {
        DrawBezier( fTmpTuple.start, fTmpTuple.middle, fTmpTuple.end, fImgMat );
    }
    // update display
    fImgQImage = Mat2QImage( fImgMat );
    fImageLabel->setPixmap( QPixmap::fromImage( fImgQImage ) );

    update(); // repaint the widget
}


// Conversion from QImage to cv::Mat and vice versa
// stackoverflow.com/questions/17127762/cvmat-to-qimage-and-back
// QImage2Mat
cv::Mat CQtImageViewer::QImage2Mat( const QImage &nSrc )
{
    cv::Mat tmp( nSrc.height(),
                nSrc.width(),
                CV_8UC3,
                ( uchar* )nSrc.bits(),
                nSrc.bytesPerLine() );
    cv::Mat result;	// deep copy
    cvtColor( tmp, result, CV_BGR2RGB );
    return result;
}

// Mat2QImage
QImage CQtImageViewer::Mat2QImage( const cv::Mat &nSrc )
{
    cv::Mat tmp;
    cvtColor( nSrc, tmp, CV_BGR2RGB); // copy and convert color space
    QImage result( ( const uchar* )tmp.data,
                    tmp.cols,
                    tmp.rows,
                    tmp.step,
                    QImage::Format_RGB888 );
    result.bits();	// enforce deep copy, see documentation of
                    // QImage::QImage( const uchar *data, int width, int height, Format format )
    return result;
}

// Constructor
CQtImageViewer::CQtImageViewer( void ) :
    fImageLabel( NULL ),
    fScrollArea( NULL ),
    fVpkg( NULL ),
    fPathOnSliceIndex( -1 ),
    fTupleIndex( 0 )
{
    fImageLabel = new QLabel;
    fImageLabel->setBackgroundRole( QPalette::Base );
    fImageLabel->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );
    fImageLabel->setScaledContents( true );

    fScrollArea = new QScrollArea;
    fScrollArea->setBackgroundRole( QPalette::Dark );
    fScrollArea->setWidget( fImageLabel );
    setCentralWidget( fScrollArea );

    CreateActions();
    CreateMenus();

    setWindowTitle( tr( "Image Viewer" ) );
    resize( 500, 400 );
}

// Constructor
CQtImageViewer::CQtImageViewer( const std::string &nVpkgPath,
                                int nPathOnSliceIndex ) :
    fImageLabel( NULL ),
    fScrollArea( NULL ),
    fVpkg( NULL ),
    fPathOnSliceIndex( -1 ),
    fTupleIndex( 0 )
{
    fImageLabel = new QLabel;
    fImageLabel->setBackgroundRole( QPalette::Base );
    fImageLabel->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );
    fImageLabel->setScaledContents( true );

    fScrollArea = new QScrollArea;
    fScrollArea->setBackgroundRole( QPalette::Dark );
    fScrollArea->setWidget( fImageLabel );
    setCentralWidget( fScrollArea );

    CreateActions();
    CreateMenus();

    setWindowTitle( tr( "Image Viewer" ) );
    resize( 500, 400 );

    // open volume package and load slice
    if ( !InitializeVolumePkg( nVpkgPath ) ) {
        printf( "ERROR: cannot open the volume package at the specified location.\n" );
        return;
    }
    if ( nPathOnSliceIndex < 2 || nPathOnSliceIndex > fVpkg->getNumberOfSlices() - 3 ) {
        printf( "ERROR: cannot load the slice at the specified index.\n" );
        return;
    } else {
        fPathOnSliceIndex = nPathOnSliceIndex;
        OpenSlice( fPathOnSliceIndex );
        InitializeView();
    }
}

// Destructor
CQtImageViewer::~CQtImageViewer( void )
{
    if ( fVpkg != NULL ) {
        delete fVpkg;
        fVpkg = NULL;
    }
}

// Open
void CQtImageViewer::Open( void )
{
    QString aVpkgPath = QString( "" );
    aVpkgPath = QFileDialog::getExistingDirectory( this,
                                                    tr( "Open Directory" ),
                                                    QDir::homePath(),
                                                    QFileDialog::ShowDirsOnly |
                                                    QFileDialog::DontResolveSymlinks );
    if ( aVpkgPath.length() == 0 ) { // canceled
        return;
    }
    if ( !InitializeVolumePkg( aVpkgPath.toStdString() + "/" ) ) {
        printf( "ERROR: cannot open the volume package at the specified location.\n" );
        return;
    }

    // get slice index
    bool aIsIndexOk = false;
    fPathOnSliceIndex = QInputDialog::getInt( this,
                                            tr( "Input" ),
                                            tr( "Please enter the index of the slice you want to draw path on: " ),
                                            2,								// default value
                                            2,								// minimum value
                                            fVpkg->getNumberOfSlices() - 3,	// maximum value
                                            1,								// step
                                            &aIsIndexOk );
    if ( !aIsIndexOk ) {
        printf( "WARNING: nothing was loaded because no slice index specified.\n" );
        return;
    }

    // read slice
    OpenSlice( fPathOnSliceIndex );

    // set up view
    InitializeView();
}

// Print
void CQtImageViewer::Print( void )
{
    Q_ASSERT( fImageLabel->pixmap() );
#ifndef QT_NO_PRINTER
    QPrintDialog aDialog( &fPrinter, this );
    if ( aDialog.exec() ) {
        QPainter aPainter( &fPrinter );
        QRect aRect = aPainter.viewport();
        QSize aSize = fImageLabel->pixmap()->size();
        aSize.scale( aRect.size(), Qt::KeepAspectRatio );
        aPainter.setViewport( aRect.x(), aRect.y(), aSize.width(), aSize.height() );
        aPainter.setWindow( fImageLabel->pixmap()->rect() );
        aPainter.drawPixmap( 0, 0, *fImageLabel->pixmap() );
    }
#endif // QT_NO_PRINTER
}

// Zoom in
void CQtImageViewer::ZoomIn( void )
{
    ScaleImage( 1.25 );
}

// Zoom out
void CQtImageViewer::ZoomOut( void )
{
    ScaleImage( 0.8 );
}

// Normal size
void CQtImageViewer::NormalSize( void )
{
    fImageLabel->adjustSize();
    fScaleFactor = 1.0;
}

// Fit to window
void CQtImageViewer::FitToWindow( void )
{
    bool aFitToWindow = fFitToWindowAct->isChecked();
    fScrollArea->setWidgetResizable( aFitToWindow );
    if ( !aFitToWindow ) {
        NormalSize();
    }
    UpdateActions();
}

// About
void CQtImageViewer::About( void )
{
    QMessageBox::about( this, tr( "About Qt Image Viewer" ),
        tr( "<p>the <b>Image Viewer</b> example shows how to combine QLabel "
            "and QScrollArea to display an image. QLabel is typically used "
            "for displaying a text, but it can also display an image. "
            "QScrollArea provides a scrolling view around another widget. "
            "If the child widget exceeds the size fo the frame, QScrollArea "
            "automatically provides scroll bards. </p><p>the example "
            "demonstrates how QLabel's ability to scale its contents "
            "(QLabel::scaledContents), and QScrollArea's ability to "
            "automatically resize its contents "
            "(QScrollArea::widgetResizeable), cna be sued to implement "
            "zomming and scaling features. </p><p>In addition the exmaple "
            "shows how to use QPainter to print an image.</p>" ) );
}

// Create actions
void CQtImageViewer::CreateActions( void )
{
    fOpenAct = new QAction( tr( "&Open..." ), this );
    fOpenAct->setShortcut( tr( "Ctrl+O" ) );
    connect( fOpenAct, SIGNAL( triggered() ), this, SLOT( Open() ) );

    fPrintAct = new QAction( tr( "&Print..." ), this );
    fPrintAct->setShortcut( tr( "Ctrl+P" ) );
    fPrintAct->setEnabled( false );
    connect( fPrintAct, SIGNAL( triggered() ), this, SLOT( Print() ) );

    fExitAct = new QAction( tr( "E&xit" ), this );
    fExitAct->setShortcut( tr( "Ctrl+Q" ) );
    connect( fExitAct, SIGNAL( triggered() ), this, SLOT( Close() ) );

    fZoomInAct = new QAction( tr( "Zoom &In (25%)" ), this );
    fZoomInAct->setShortcut( tr( "Ctrl++" ) );
    fZoomInAct->setEnabled( false );
    connect( fZoomInAct, SIGNAL( triggered() ), this, SLOT( ZoomIn() ) );

    fZoomOutAct = new QAction( tr( "Zoom &Out (25%)" ), this );
    fZoomOutAct->setShortcut( tr( "Ctrl+-" ) );
    fZoomOutAct->setEnabled( false );
    connect( fZoomOutAct, SIGNAL( triggered() ), this, SLOT( ZoomOut() ) );

    fNormalSizeAct = new QAction( tr( "&Normal Size" ), this );
    fNormalSizeAct->setShortcut( tr( "Ctrl+S" ) );
    fNormalSizeAct->setEnabled( false );
    connect( fNormalSizeAct, SIGNAL( triggered() ), this, SLOT( NormalSize() ) );

    fFitToWindowAct = new QAction( tr( "&Fit to Window" ), this );
    fFitToWindowAct->setEnabled( false );
    fFitToWindowAct->setCheckable( true );
    fFitToWindowAct->setShortcut( tr( "Ctrl+F" ) );
    connect( fFitToWindowAct, SIGNAL( triggered() ), this, SLOT( FitToWindow() ) );

    fAboutAct = new QAction( tr( "&About" ), this );
    connect( fAboutAct, SIGNAL( triggered() ), this, SLOT( About() ) );

    fAboutQtAct = new QAction( tr( "About &Qt" ), this );
//    connect( fAboutQtAct, SIGNAL( triggered() ), QCoreApplication::instance(), SLOT( aboutQt() ) );
    connect( fAboutQtAct, SIGNAL( triggered() ), this, SLOT( AboutQt() ) );
}

// Create menus
void CQtImageViewer::CreateMenus( void )
{
    fFileMenu = new QMenu( tr( "&File" ), this );
    fFileMenu->addAction( fOpenAct );
    fFileMenu->addAction( fPrintAct );
    fFileMenu->addSeparator();
    fFileMenu->addAction( fExitAct );

    fViewMenu = new QMenu( tr( "&View" ), this );
    fViewMenu->addAction( fZoomInAct );
    fViewMenu->addAction( fZoomOutAct );
    fViewMenu->addAction( fNormalSizeAct );
    fViewMenu->addSeparator();
    fViewMenu->addAction( fFitToWindowAct );

    fHelpMenu = new QMenu( tr( "&Help" ), this );
    fHelpMenu->addAction( fAboutAct );
    fHelpMenu->addAction( fAboutQtAct );

    menuBar()->addMenu( fFileMenu );
    menuBar()->addMenu( fViewMenu );
    menuBar()->addMenu( fHelpMenu );
}

// Update actions
void CQtImageViewer::UpdateActions( void )
{
    fZoomInAct->setEnabled( !fFitToWindowAct->isChecked() );
    fZoomOutAct->setEnabled( !fFitToWindowAct->isChecked() );
    fNormalSizeAct->setEnabled( !fFitToWindowAct->isChecked() );
}

// Scale image
void CQtImageViewer::ScaleImage( double nFactor )
{
    Q_ASSERT( fImageLabel->pixmap() );
    fScaleFactor *= nFactor;
    fImageLabel->resize( fScaleFactor * fImageLabel->pixmap()->size() );

    AdjustScrollBar( fScrollArea->horizontalScrollBar(), nFactor );
    AdjustScrollBar( fScrollArea->verticalScrollBar(), nFactor );

    fZoomInAct->setEnabled( fScaleFactor < 3.0 );
    fZoomOutAct->setEnabled( fScaleFactor > 0.333 );
}

// AdjustScrollBar
void CQtImageViewer::AdjustScrollBar( QScrollBar *nScrollBar,
                                        double nFactor )
{
    nScrollBar->setValue( int( nFactor * nScrollBar->value() + ( ( nFactor - 1 ) * nScrollBar->pageStep() / 2 ) ) );
}

// Close
void CQtImageViewer::Close( void )
{
    close();
}

// About Qt
void CQtImageViewer::AboutQt( void )
{
    // parent is NULL (0) so the dialog box is modal relative to the application
    QMessageBox::aboutQt( NULL );
}

// get scroll bar pixel value
double CQtImageViewer::GetScrollPixValue( const QScrollBar *nScrollBar )
{
    return nScrollBar->value();
}

void CQtImageViewer::WidgetLoc2ImgLoc( const cv::Vec2f &nWidgetLoc,
                                        cv::Vec2f &nImgLoc )
{
    float x = nWidgetLoc[ 0 ];	// horizontal coordiante
    float y = nWidgetLoc[ 1 ];	// vertical coordinate

    x -= fScrollArea->frameWidth();	// minus border frame width
    y -= fScrollArea->frameWidth();	// minus border frame width
    y -= menuBar()->height();		// minus menu bar height

    x += GetScrollPixValue( fScrollArea->horizontalScrollBar() );
    y += GetScrollPixValue( fScrollArea->verticalScrollBar() );

    // take image scale factor into account
    x /= fScaleFactor;
    y /= fScaleFactor;

    nImgLoc[ 0 ] = x;
    nImgLoc[ 1 ] = y;
}

// note: this program generate Bezier curve from user clicks
//       It will draw curve after 3 clicks, the start and end poitns are anchor.
//       Then use the last end point as the new start point.

// handle mouse event
void CQtImageViewer::mousePressEvent( QMouseEvent *nEvent )
{
    if ( fVpkg == NULL ) {
        QMessageBox::information( this,
                                    tr( "Prompt" ),
                                    tr( "Please select a valid volume package and slice index that you want to draw on. \nYou can do this in \"File->Open\"." ) );
        return;
    }

    cv::Vec2f aWidgetLoc, aImgLoc;
    aWidgetLoc[ 0 ] = nEvent->x();	// horizontal coordiante
    aWidgetLoc[ 1 ] = nEvent->y();	// vertical coordiante

    WidgetLoc2ImgLoc( aWidgetLoc, aImgLoc );

    nEvent->accept();

#ifdef _DEBUG
    // REVISIT - for debug purpose
    QMessageBox::information( this,
                                tr( "Click" ),
                                tr( "Widget location: %1, %2; Image location: %3, %4" )
                                    .arg( nEvent->x() ).arg( nEvent->y() ).arg( x ).arg( y ) );
#endif // _DEBUG

    if ( nEvent->buttons() & Qt::LeftButton ) {
        // REVISIT - fill me here
//		cout << "x: " << x << ", y: " << y << endl;
        if ( fTupleIndex == 0 ) {
            fTmpTuple.start[ 0 ] = aImgLoc[ 0 ];
            fTmpTuple.start[ 1 ] = aImgLoc[ 1 ];
        } else if ( fTupleIndex == 1 ) {
            fTmpTuple.middle[ 0 ] = aImgLoc[ 0 ];
            fTmpTuple.middle[ 1 ] = aImgLoc[ 1 ];

            fTmpTuple.end[ 0 ] = aImgLoc[ 0 ]; // prevent uninitialized value
            fTmpTuple.end[ 1 ] = aImgLoc[ 1 ];
        } else if ( fTupleIndex == 2 ) {
            fTmpTuple.end[ 0 ] = aImgLoc[ 0 ];
            fTmpTuple.end[ 1 ] = aImgLoc[ 1 ];

            fPath.push_back( fTmpTuple );
            fTupleIndex = ( fTupleIndex + 1 ) % 3;

            // start a new segment with the last end as the new start
            fTmpTuple.start[ 0 ] = aImgLoc[ 0 ];
            fTmpTuple.start[ 1 ] = aImgLoc[ 1 ];
        }
        fTupleIndex = ( fTupleIndex + 1 ) % 3;

//		cout << "Index: " << gTupleIndex << endl;
    } else if ( nEvent->buttons() & Qt::RightButton ) {
        printf( "Save path to file and exit\n" );
        // REVISIT - should check if we are in the middle of a creating a Bezier curve
        SavePath( fVpkg,
                    fPath,
                    fPathOnSliceIndex );
        QApplication::quit(); // use quit() instead of exit() to terminate decently, check MacOSX
    }

    UpdateView();
}

void CQtImageViewer::mouseMoveEvent( QMouseEvent *nEvent )
{
    if ( fTupleIndex == 2 ) {
        // REVISIT - should also do widget coordinates to image coordinates conversion
        cv::Vec2f aWidgetLoc, aImgLoc;
        aWidgetLoc[ 0 ] = nEvent->x();	// horizontal coordiante
        aWidgetLoc[ 1 ] = nEvent->y();	// vertical coordiante

        WidgetLoc2ImgLoc( aWidgetLoc, aImgLoc );

        fTmpTuple.end[ 0 ] = aImgLoc[ 0 ];
        fTmpTuple.end[ 1 ] = aImgLoc[ 1 ];
    }

    nEvent->accept();

    UpdateView();
}

void CQtImageViewer::wheelEvent( QWheelEvent *nWheelEvent )
{
    int aNumDegrees = nWheelEvent->delta() / 8;
    int aNumSteps = aNumDegrees / 15;

    Qt::KeyboardModifiers aMod = nWheelEvent->modifiers();
    if ( aMod & Qt::ControlModifier ) {	// zoom in/out
        if ( nWheelEvent->orientation() == Qt::Vertical ) {
            if ( aNumSteps > 0 ) {
                ZoomIn();
            } else {
                ZoomOut();
            }
        }
    } else {
        if ( nWheelEvent->orientation() == Qt::Horizontal ) {
            fScrollArea->horizontalScrollBar()->setValue( fScrollArea->horizontalScrollBar()->value() + aNumSteps );
        } else {
            fScrollArea->verticalScrollBar()->setValue( fScrollArea->verticalScrollBar()->value() + aNumSteps );
        }
    }

    nWheelEvent->accept();
}
