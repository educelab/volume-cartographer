// CVolumeViewerWithCurve.cpp
// Chao Du 2015 April
#include "CVolumeViewerWithCurve.h"
#include "UDataManipulateUtils.h"

using namespace ChaoVis;

// Constructor
CVolumeViewerWithCurve::CVolumeViewerWithCurve( void ) :
    fCurveRef( NULL )
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
void CVolumeViewerWithCurve::SetCurve( CBSpline &nCurve )
{
    fCurveRef = &nCurve;
}

// Update the B-spline curve
void CVolumeViewerWithCurve::UpdateCurve( void )
{
    if ( fCurveRef != NULL ) {
         fCurveRef->SetControlPoints( fControlPoints );
    }
}

// Update the view
void CVolumeViewerWithCurve::UpdateView( void )
{
    fImgMatCache.copyTo( fImgMat );

    if ( fCurveRef != NULL ) {
        fCurveRef->DrawOnImage( fImgMat, cv::Scalar( 0, 0, 255 ) );
    }

    for ( size_t i = 0; i < fControlPoints.size(); ++i ) {
        cv::circle( fImgMat, cv::Point2f( fControlPoints[ i ] ), 1, cv::Scalar( 255, 0, 0 ) );
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

    if ( event->buttons() & Qt::LeftButton ) { // add points

        fControlPoints.push_back( aImgLoc );

        if ( fControlPoints.size() > 2 ) {
            UpdateCurve();
        }

    } else if ( event->buttons() & Qt::RightButton ) { // finish curve

        // REVISIT - save path

    } else { // other mouse press events
        // do nothing
    }

    UpdateView();
    event->accept();
}

// Handle mouse move event
void CVolumeViewerWithCurve::mouseMoveEvent( QMouseEvent *event )
{
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
