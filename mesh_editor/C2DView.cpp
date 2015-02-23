// C2DView.cpp
// Chao Du 2014 Dec
#include "C2DView.h"

#include "HBase.h"

#include "CXCurve.h"
#include "CMeshGL.h"


using namespace ChaoVis;

C2DView::C2DView( QWidget *parent ) :
	QWidget( parent ),
	fImage( NULL ),
	fImageLabel( NULL ),
	fScrollArea( NULL ),
	fZoomInBtn( NULL ),
	fZoomOutBtn( NULL ),
	fResetBtn( NULL ),
	fNextBtn( NULL ),
	fPrevBtn( NULL ),
	fScaleFactor( 1.0 ),
	fCurve( NULL ),
	fMeshModelRef( NULL ),
	fVertexIsChanged( false ),
    fSelectedPointIndex( -1 ),
    fCurrentSliceIndex( -1 )
{
	// buttons
	fZoomInBtn = new QPushButton( tr( "Zoom In" ), this );
	fZoomOutBtn = new QPushButton( tr( "Zoom Out" ), this );
	fResetBtn = new QPushButton( tr( "Reset" ), this );
	fNextBtn = new QPushButton( tr( "Next Slice" ), this );
	fPrevBtn = new QPushButton( tr( "Previous Slice" ), this );
    // text edit
    // REVISIT - you may want to refactor this to a "CreateSingleLineTextEdit()" function
    fSliceIndexEdit = new QTextEdit( this );
    fSliceIndexEdit->setText( tr( "0" ) );
    fSliceIndexEdit->setEnabled( false );
    fSliceIndexEdit->setWordWrapMode( QTextOption::NoWrap );
    fSliceIndexEdit->setHorizontalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
    fSliceIndexEdit->setVerticalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
    QFontMetrics fm( font() );
    int h = qMax( fm.height(), 14 ) + 4;
    int w = fm.width( QLatin1Char( 'x' ) ) * 17 + 4;
    QStyleOptionFrameV2 opt;
    opt.initFrom( this );
    fSliceIndexEdit->setFixedHeight( style()->sizeFromContents( QStyle::CT_LineEdit, &opt, QSize( w, h ).expandedTo( QApplication::globalStrut()), this ).height() );
	
	QHBoxLayout *aButtonsLayout = new QHBoxLayout;
	aButtonsLayout->addWidget( fZoomInBtn );
	aButtonsLayout->addWidget( fZoomOutBtn );
	aButtonsLayout->addWidget( fResetBtn );
	aButtonsLayout->addWidget( fPrevBtn );
	aButtonsLayout->addWidget( fNextBtn );
    aButtonsLayout->addWidget( fSliceIndexEdit );

	connect( fZoomInBtn, SIGNAL( clicked() ), this, SLOT( OnZoomInClicked() ) );
	connect( fZoomOutBtn, SIGNAL( clicked() ), this, SLOT( OnZoomOutClicked() ) );
	connect( fResetBtn, SIGNAL( clicked() ), this, SLOT( OnResetClicked() ) );
	connect( fNextBtn, SIGNAL( clicked() ), this, SLOT( OnNextClicked() ) );
	connect( fPrevBtn, SIGNAL( clicked() ), this, SLOT( OnPrevClicked() ) );

	// scroll aread and image label
	fImageLabel = new QLabel;
	fImageLabel->setBackgroundRole( QPalette::Base );
	fImageLabel->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );
	fImageLabel->setScaledContents( true );
	
	fScrollArea = new QScrollArea;
	fScrollArea->setBackgroundRole( QPalette::Dark );
	fScrollArea->setWidget( fImageLabel );

	QVBoxLayout *aWidgetLayout = new QVBoxLayout;
	aWidgetLayout->addWidget( fScrollArea );
	aWidgetLayout->addLayout( aButtonsLayout );

	setLayout( aWidgetLayout );

	UpdateButtons();
}

C2DView::~C2DView( void )
{
	deleteNULL( fImage );
	deleteNULL( fImageLabel );
	deleteNULL( fScrollArea );
	deleteNULL( fZoomInBtn );
	deleteNULL( fZoomOutBtn );
	deleteNULL( fResetBtn );

    // NOTE: since we only hold a reference (pointer) to the actual curve data,
    //       we don't need to delete the content now.
//	deleteNULL( fCurve );
	// NOTE: don't need to delete fMeshModelRef since it's only a reference
}

// Set image
void C2DView::SetImage( const QImage &nSrc )
{
	if ( fImage == NULL ) {
		fImage = new QImage( nSrc );
	} else {
		*fImage = nSrc;
	}

	fImageLabel->setPixmap( QPixmap::fromImage( *fImage ) );
	fImageLabel->resize( fScaleFactor * fImageLabel->pixmap()->size() );

	update(); // repaint the widget
	UpdateButtons();
}

// Set intersection curve
void C2DView::SetIntersection( CXCurve* nCurve )
{
//	if ( fCurve != NULL ) {
//		delete fCurve;
//	}
//	fCurve = new CXCurve( *nCurve );
    fCurve = nCurve;
}

// Set intersection curve, and also the neighboring curves
void C2DView::SetIntersection( const std::vector< CXCurve* > &nCurves,
                                int nIndex )
{
    // REVISIT - assigning the actual curves may be wasteful, consider
    //           changing this to assigning the reference of the curves and index
    fCurve = nCurves[ nIndex ];
    // REVISIT - clean this magic number 17!
    fCurvesLower.clear();
    fCurvesUpper.clear();
    for ( int i = 0; i < ( 17 - 1 ) / 2; ++i ) {
        // lower
        if ( nIndex - i - 1 > -1 ) {
            fCurvesLower.push_back( nCurves[ nIndex - i - 1 ] );
        }
        // upper
        if ( nIndex + i + 1 < nCurves.size() ) {
            fCurvesUpper.push_back( nCurves[ nIndex + i + 1 ] );
        }
    }
}

// Set current slice index
void C2DView::SetSliceIndex( int nCurrentSliceIndex )
{
    fCurrentSliceIndex = nCurrentSliceIndex;
    char aSliceIndexTemplate[ 16 ] = "%d";
    char aSliceIndex[ 16 ];
    sprintf( aSliceIndex, aSliceIndexTemplate, fCurrentSliceIndex );
    fSliceIndexEdit->setText( tr( aSliceIndex ) );
}

// Set the reference to mesh model
void C2DView::SetMeshModel( CMeshGL *nMeshModel )
{
    fMeshModelRef = nMeshModel;
}

// Handle mouse press event
void C2DView::mousePressEvent( QMouseEvent *event )
{
	// REVISIT - design: we use left click to select vertex, then drag and drop
	fLastPos = event->pos();

	// offset by child's position
    QPoint aOffset = fImageLabel->mapTo( this, QPoint( 0, 0 ) );
    fLastPos -= aOffset;

    // REVISIT - now we take the scale and scrolling into account.
    //           the clicked position will be transformed to the location
    //           where the absolute image coordinate is.
    fLastPos = ConvertWidgetPosToImagePos( fLastPos );

	// REVISIT - select a point to be manipulated
	if ( fCurve != NULL ) {
		fSelectedPointIndex = SelectPointOnCurve( fCurve, fLastPos );
	}
}

// Handle mouse move event
void C2DView::mouseMoveEvent( QMouseEvent *event )
{
    // offset by child's position
    QPoint aOffset = fImageLabel->mapTo( this, QPoint( 0, 0 ) );

    QPoint aNewPosOnImg = ConvertWidgetPosToImagePos( event->pos() - aOffset );

    int aDx = aNewPosOnImg.x() - fLastPos.x();
    int aDy = aNewPosOnImg.y() - fLastPos.y();

	// REVISIT - should take scale factor into account
	if ( fSelectedPointIndex != -1 ) {
		// update curve
        //Vec2< int > aOldPt( fCurve->GetPoint( fSelectedPointIndex )[ 0 ], fCurve->GetPoint( fSelectedPointIndex )[ 1 ] );
        //fCurve->SetPoint( fSelectedPointIndex, Vec2< float >( aOldPt[ 0 ] + aDx, aOldPt[ 1 ] + aDy ) );
        // REVISIT - instead of changing the point directly, we pass the difference and change the point and its neighbor

//#define _DEBUG
#ifdef _DEBUG
        qDebug() << "Pos: " << aNewPosOnImg.x() << " " << aNewPosOnImg.y() << "\n";
        qDebug() << "D: " << aDx << " " << aDy << "\n";
        qDebug() << "Point: " << fCurve->GetPoint( fSelectedPointIndex )[ 0 ] << " " << fCurve->GetPoint( fSelectedPointIndex )[ 1 ] << "\n";
#endif // _DEBUG
//#undef _DEBUG
        fCurve->SetPointByDifference( fSelectedPointIndex,
                                      Vec2< float >( aDx, aDy ),
                                      CosineImpactFunc,
                                      17 );// fCurve->GetPointsNum() / 2 ); // REVISIT - impact range should be adjustable
        // REVISIT - change neighboring curves, assume the point index of different curves are the same (actually, they may not be so)
        // fCurrentSliceIndex - 1, fCurrentSliceIndex - 2, ...
        float aWeight = 1.0;
        for ( size_t i = 0; i < fCurvesLower.size(); ++i ) {
            if ( fCurvesLower[ i ] != NULL ) {
                aWeight = CosineImpactFunc( 1.0,
                                            i + 1,
                                            ( 17 + 1 ) / 2 );
                fCurvesLower[ i ]->SetPointByDifference( fSelectedPointIndex,
                                              Vec2< float >( aDx, aDy ) * aWeight,
                                              CosineImpactFunc,
                                              17 );// fCurve->GetPointsNum() / 2 ); // REVISIT - impact range should be adjustable
            }
        }
        // fCurrentSliceIndex + 1, fCurrentSliceIndex + 2, ...
        for ( size_t i = 0; i < fCurvesUpper.size(); ++i ) {
            if ( fCurvesUpper[ i ] != NULL ) {
                aWeight = CosineImpactFunc( 1.0,
                                            i + 1,
                                            ( 17 + 1 ) / 2 );
                fCurvesUpper[ i ]->SetPointByDifference( fSelectedPointIndex,
                                              Vec2< float >( aDx, aDy ) * aWeight,
                                              CosineImpactFunc,
                                              17 );// fCurve->GetPointsNum() / 2 ); // REVISIT - impact range should be adjustable
            }
        }

		// update view
		update();

        fLastPos = aNewPosOnImg;
		fVertexIsChanged = true;
	}
}

// Handle mouse release event
void C2DView::mouseReleaseEvent( QMouseEvent *event )
{
	// REVISIT - FILL ME HERE
	// REVISIT - TODO: notify 3D model to change vertex accordingly
	// REVISIT - fCurve != NULL and fMeshModelRef != NULL ought to be equivalent
	if ( fCurve != NULL && fMeshModelRef != NULL && fVertexIsChanged ) {
		// REVISIT - design: drag and drop to manipulate 2D curve, then change the mesh
		//                   the texture is generated again at last
		// REVISIT - change mesh vertex, this need careful design
		//           after this, the 3D view should be notified for update

        //fMeshModelRef->ChangeVertex( fCurve->Get3DIndex( fSelectedPointIndex ),
        //                             Vec3< float >( fCurrentSliceIndex, fLastPos.x(), fLastPos.y() ) );
        // REVISIT - instead of updating only one point, we update the whole curve
        fMeshModelRef->ChangeVertex( fCurve, fCurrentSliceIndex - 1 ); // REVISIT - -1 because the slices in vpkg starts from 1
        // REVISIT - change neighboring curves
        // fCurrentSliceIndex - 1, fCurrentSliceIndex - 2, ...
        for ( size_t i = 0; i < fCurvesLower.size(); ++i ) {
            if ( fCurvesLower[ i ] != NULL ) {
                fMeshModelRef->ChangeVertex( fCurvesLower[ i ], fCurrentSliceIndex - 1 - i - 1 );
            }
        }
        // fCurrentSliceIndex + 1, fCurrentSliceIndex + 2, ...
        for ( size_t i = 0; i < fCurvesUpper.size(); ++i ) {
            if ( fCurvesUpper[ i ] != NULL ) {
                fMeshModelRef->ChangeVertex( fCurvesUpper[ i ], fCurrentSliceIndex - 1 + i + 1 );
            }
        }

        // since we hold the reference of the data in CWindow, the mesh is updated, and 3D view is refreshed
        emit SendSignalMeshChanged();
	}

	// clear states
	fVertexIsChanged = false;
    fSelectedPointIndex = -1;
}

// Handle paint event
void C2DView::paintEvent( QPaintEvent *event )
{
	if ( fCurve != NULL ) {
		// REVISIT - DrawCurve draws the curve on fImage, which may be ambiguious
		DrawCurve();
	}
}

// Handle resize event
void C2DView::resizeEvent( QResizeEvent *event )
{
	// REVISIT - FILL ME HERE
}

// Handle zoom in click
void C2DView::OnZoomInClicked( void )
{
	ScaleImage( 1.25 );
}

// Handle zoom out click
void C2DView::OnZoomOutClicked( void )
{
	ScaleImage( 0.8 );
}

// Handle reset click
void C2DView::OnResetClicked( void )
{
	fImageLabel->adjustSize();
	fScaleFactor = 1.0;

	UpdateButtons();
}

// Handle next slice click
void C2DView::OnNextClicked( void )
{
	// REVISIT - FILL ME HERE
	// TODO: (1) load new slice 
	//       (2) update intersection curve 
	//       (3) notify 3D view to update the location of the slice plane proxy
	// placeholder
	//QMessageBox::information( this, tr( "clicked" ), tr( "NextSlice clicked" ) );
	// REVISIT - we notify CWindow to do the above things
	emit SendSignalOnNextClicked();
}

// Handle previous slice click
void C2DView::OnPrevClicked( void )
{
	// REVISIT - FILL ME HERE
	// TODO: (1) load new slice 
	//       (2) update intersection curve 
	//       (3) notify 3D view to update the location of the slice plane proxy
	// placeholder
	//QMessageBox::information( this, tr( "clicked" ), tr( "PrevSlice clicked" ) );

	emit SendSignalOnPrevClicked();
}

// Scale image
void C2DView::ScaleImage( double nFactor )
{
	Q_ASSERT( fImageLabel->pixmap() );

	fScaleFactor *= nFactor;
	fImageLabel->resize( fScaleFactor * fImageLabel->pixmap()->size() );

	AdjustScrollBar( fScrollArea->horizontalScrollBar(), nFactor );
	AdjustScrollBar( fScrollArea->verticalScrollBar(), nFactor );

	// restrict scale range
	UpdateButtons();
}

// Adjust scroll bar of scroll area
void C2DView::AdjustScrollBar( QScrollBar *nScrollBar,
							   double nFactor )
{
	nScrollBar->setValue( int( nFactor * nScrollBar->value() + 
		( ( nFactor - 1 ) * nScrollBar->pageStep() / 2 ) ) );
}

// Update the status of buttons
void C2DView::UpdateButtons( void )
{
	fZoomInBtn->setEnabled( fImage != NULL && fScaleFactor < 3.0 );
	fZoomOutBtn->setEnabled( fImage != NULL && fScaleFactor > 0.3333 );
	fResetBtn->setEnabled( fImage != NULL && fabs( fScaleFactor - 1.0 ) > 1e-6 );
	fNextBtn->setEnabled( fImage != NULL );
	fPrevBtn->setEnabled( fImage != NULL );
}

// Draw curve
void C2DView::DrawCurve( void )
{
	if ( fImage != NULL && fCurve != NULL ) {
		QImage aImgCache( *fImage );
		QPainter aPainter( &aImgCache );

        aPainter.setPen( Qt::green );
        for ( int i = 0; i < fCurve->GetPointsNum() - 1; ++i ) { // REVISIT - when fCurve->GetPointsNum() is 0, using size_t and subtract 1 will under flow.
			aPainter.drawLine( QPointF( fCurve->GetPoint( i )[ 0 ], fCurve->GetPoint( i )[ 1 ] ), 
							   QPointF( fCurve->GetPoint( i + 1 )[ 0 ], fCurve->GetPoint( i + 1 )[ 1 ] ) );
		}

		aPainter.setPen( Qt::red );
        for ( size_t i = 0; i < fCurve->GetPointsNum(); ++i ) {
            aPainter.drawPoint( QPointF( fCurve->GetPoint( i )[ 0 ], fCurve->GetPoint( i )[ 1 ] ) );
        }

		fImageLabel->setPixmap( QPixmap::fromImage( aImgCache ) );
	}
}

// Select point on curve
int C2DView::SelectPointOnCurve( const CXCurve *nCurve,
								const QPoint &nPt )
{
    float DIST_THRESHOLD = 2.0; // REVISIT - image scale should be taken into account,
	                            //           and the constant should be declared somewhere else
	for ( size_t i = 0; i < nCurve->GetPointsNum(); ++i ) {
		if ( Norm< float >( Vec2< float >( nCurve->GetPoint( i )[ 0 ] - nPt.x(), nCurve->GetPoint( i )[ 1 ] - nPt.y() ) ) < DIST_THRESHOLD ) {
			return i;
		}
	}
	return -1;
}

// Convert the postion on widget to the position on image, taking scale and scrolling into account
QPoint C2DView::ConvertWidgetPosToImagePos( const QPoint &nPos )
{
    float x = nPos.x();
    float y = nPos.y();

    // take scroll value into account
    // REVISIT - the "mapTo()" function has already taken care of the scroll value
    //           when you convert the origin of the image, so we don't need these lines
//    x += GetScrollPixValue( fScrollArea->horizontalScrollBar() );
//    y += GetScrollPixValue( fScrollArea->verticalScrollBar() );

    // take scale factor into account
    x /= fScaleFactor;
    y /= fScaleFactor;

    return QPoint( round( x ), round( y ) );
}

// Get the current value of a scroll bar
double C2DView::GetScrollPixValue( const QScrollBar *nScrollBar )
{
    return nScrollBar->value();
}
