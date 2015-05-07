// CVolumeViewerWithCurve.cpp
// Chao Du 2015 April
#include "CVolumeViewerWithCurve.h"
#include "UDataManipulateUtils.h"

using namespace ChaoVis;

// Constructor
CVolumeViewerWithCurve::CVolumeViewerWithCurve( void ) :
    fSplineCurveRef( NULL ),
    fIntersectionCurveRef( NULL ),
    fViewState( EViewState::ViewStateIdle ),
    fSelectedPointIndex( -1 ),
    fVertexIsChanged( false )
{
}

// Destructor
CVolumeViewerWithCurve::~CVolumeViewerWithCurve( void )
{
}

// Set image
void CVolumeViewerWithCurve::SetImage( const QImage &nSrc )
{
    if ( fImgQImage == NULL ) {
        fImgQImage = new QImage( nSrc );
    } else {
        *fImgQImage = nSrc;
    }

    fCanvas->setPixmap( QPixmap::fromImage( *fImgQImage ) );
    fCanvas->resize( fScaleFactor * fCanvas->pixmap()->size() );

    fImgMat = QImage2Mat( *fImgQImage );
    fImgMat.copyTo( fImgMatCache );

    update();
    UpdateButtons();
}

// Set the curve, we only hold a pointer to the original one so the data can be synchronized
void CVolumeViewerWithCurve::SetSplineCurve( CBSpline &nCurve )
{
    fSplineCurveRef = &nCurve;
}

// Update the B-spline curve
void CVolumeViewerWithCurve::UpdateSplineCurve( void )
{
    if ( fSplineCurveRef != NULL ) {
         fSplineCurveRef->SetControlPoints( fControlPoints );
    }
}

// Update the view
void CVolumeViewerWithCurve::UpdateView( void )
{
    fImgMatCache.copyTo( fImgMat );

    if ( fViewState == EViewState::ViewStateDraw ) {
        if ( fSplineCurveRef != NULL ) {
            fSplineCurveRef->DrawOnImage( fImgMat, cv::Scalar( 0, 0, 255 ) );
        }

        for ( size_t i = 0; i < fControlPoints.size(); ++i ) {
            cv::circle( fImgMat, cv::Point2f( fControlPoints[ i ] ), 1, cv::Scalar( 255, 0, 0 ) );
        }
    } else if ( fViewState == EViewState::ViewStateEdit ) {
        if ( fIntersectionCurveRef != NULL ) {
            DrawIntersectionCurve();
        }
    } else {
        // idle state, do nothing
    }

    *fImgQImage = Mat2QImage( fImgMat );

    fCanvas->setPixmap( QPixmap::fromImage( *fImgQImage ) );
    fCanvas->resize( fScaleFactor * fCanvas->pixmap()->size() );

    update(); // repaint the widget
}

// Handle mouse press event
void CVolumeViewerWithCurve::mousePressEvent( QMouseEvent *event )
{
    cv::Vec2f aWidgetLoc, aImgLoc;
    aWidgetLoc[ 0 ] = event->x(); // horizontal coordinate
    aWidgetLoc[ 1 ] = event->y(); // vertical coordinate

    WidgetLoc2ImgLoc( aWidgetLoc, aImgLoc );

    if ( fViewState == EViewState::ViewStateDraw ) {
        if ( event->buttons() & Qt::LeftButton ) { // add points

            fControlPoints.push_back( aImgLoc );

            if ( fControlPoints.size() > 2 ) {
                UpdateSplineCurve();
            }

        } else if ( event->buttons() & Qt::RightButton ) { // finish curve

            // REVISIT - save path

        } else { // other mouse press events
            // do nothing
        }
    } else if ( fViewState == EViewState::ViewStateEdit ) {
        // REVISIT - FILL ME HERE
        if ( fIntersectionCurveRef != NULL ) {
            fSelectedPointIndex = SelectPointOnCurve( fIntersectionCurveRef, aImgLoc );
        }
    } else {
        // idle state, do nothing
    }

    UpdateView();
    event->accept();
}

// Handle mouse move event
void CVolumeViewerWithCurve::mouseMoveEvent( QMouseEvent *event )
{
    cv::Vec2f aWidgetLoc, aImgLoc;
    aWidgetLoc[ 0 ] = event->x(); // horizontal coordinate
    aWidgetLoc[ 1 ] = event->y(); // vertical coordinate

    WidgetLoc2ImgLoc( aWidgetLoc, aImgLoc );

    int aDx = aImgLoc[ 0 ] - fLastPos.x();
    int aDy = aImgLoc[ 1 ] - fLastPos.y();

    if ( fViewState == EViewState::ViewStateDraw ) {
        // REVISIT - FILL ME HERE
    } else if ( fViewState == EViewState::ViewStateEdit ) {
        if ( aDx != 0 || aDy != 0 ) {
            fIntersectionCurveRef->SetPointByDifference( fSelectedPointIndex,
                                                         Vec2< float >( aDx, aDy ),
                                                         CosineImpactFunc,
                                                         fImpactRange );
            fVertexIsChanged = true;
        }
    } else {
        // idle state, do nothing
    }

    fLastPos = QPoint( aImgLoc[ 0 ], aImgLoc[ 1 ] );

    // update view
    update();
}

// Handle mouse release event
void CVolumeViewerWithCurve::mouseReleaseEvent( QMouseEvent *event )
{
    if ( fViewState == EViewState::ViewStateEdit &&
         fIntersectionCurveRef != NULL &&
         fPathCloudRef &&
         fVertexIsChanged ) {

        // update the point positions in the path point cloud
        UpdatePathCloud();

        fVertexIsChanged = false;
        fSelectedPointIndex = -1;
    }
}

// Handle paint event
void CVolumeViewerWithCurve::paintEvent( QPaintEvent *event )
{
}

// Convert widget location to image location
void CVolumeViewerWithCurve::WidgetLoc2ImgLoc( const cv::Vec2f &nWidgetLoc,
                                               cv::Vec2f       &nImgLoc )
{
    float x = nWidgetLoc[ 0 ]; // horizontal coordinate
    float y = nWidgetLoc[ 1 ]; // vertical coordinate

    QPoint aP = fCanvas->pos(); // the image position within its parent, the scroll area
    QWidget *aCurWidget = ( QWidget * )fCanvas->parent();
    while ( aCurWidget != this ) { // the widget location from mouse event is relative
                                             // to this widget, so we stop here
        aP = aCurWidget->mapToParent( aP );
        aCurWidget = ( QWidget * )( aCurWidget->parent() );
    }

    x -= aP.x();
    y -= aP.y();

    // REVISIT - do we need to consider frame width or menu bar height? or scroll bar value?

    // take image scale factor into account
    x /= fScaleFactor;
    y /= fScaleFactor;

    nImgLoc[ 0 ] = x;
    nImgLoc[ 1 ] = y;
}

// Select point on curve
int CVolumeViewerWithCurve::SelectPointOnCurve( const CXCurve   *nCurve,
                                                const cv::Vec2f &nPt )
{
    const float DIST_THRESHOLD = 2.0; // REVISIT - image scale should be taken into account

    for ( size_t i = 0; i < nCurve->GetPointsNum(); ++i ) {
        if ( Norm< float >( Vec2< float >( nCurve->GetPoint( i )[ 0 ] - nPt[ 0 ], nCurve->GetPoint( i )[ 1 ] - nPt[ 1 ] ) ) < DIST_THRESHOLD ) {
            return i;
        }
    }
    return -1;
}

// Update the path point cloud
void CVolumeViewerWithCurve::UpdatePathCloud( void )
{
    for ( size_t i = 0; i < fPathCloudRef->size(); ++i ) {
        fPathCloudRef->points[ i ].y = fIntersectionCurveRef->GetPoint( i )[ 0 ];
        fPathCloudRef->points[ i ].z = fIntersectionCurveRef->GetPoint( i )[ 1 ];
    }
}

// Draw intersection curve on the slice
void CVolumeViewerWithCurve::DrawIntersectionCurve( void )
{
    if ( fIntersectionCurveRef != NULL ) {
        for ( size_t i = 0; i < fControlPoints.size(); ++i ) {
            cv::circle( fImgMat, cv::Point2f( fIntersectionCurveRef->GetPoint( i )[ 0 ], fIntersectionCurveRef->GetPoint( i )[ 1 ] ), 1, cv::Scalar( 255, 0, 0 ) );
        }
    }
}
