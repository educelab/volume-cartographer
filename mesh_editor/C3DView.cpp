// C3DView.cpp
// Chao Du 2014 Dec
#include "HBase.h"

#include "C3DView.h"
#include <QtCore>
#include <QtGui>
#include <Qt/QtOpenGL>

#include "CMeshGL.h"

using namespace ChaoVis;

static void NormalizeAngle( int &nAngle )
{
    while ( nAngle < 0 ) {
        nAngle += 360 * 16;
    }
    while ( nAngle > 360 * 16 ) {
        nAngle -= 360 * 16;
    }
}

// Constructor
C3DView::C3DView( QWidget *parent ) :
    QGLWidget( QGLFormat( /*QGL::SampleBuffers |*/ QGL::DoubleBuffer | QGL::DoubleBuffer | QGL::Rgba ), parent ),
	fXRot( 0 ), fYRot( 0 ), fZRot( 0 ),
	fMeshModel( NULL ),
	fPlane( NULL )
{
}

// Destructor
C3DView::~C3DView( void )
{
	deleteNULL( fMeshModel );
	deleteNULL( fPlane );
}

// Initialize mesh
bool C3DView::InitializeMeshModel( const std::string &nModelFileName )
{
    // read mesh model
	deleteNULL( fMeshModel );
    fMeshModel = new CMeshGL();
    if ( !fMeshModel->ReadModel( nModelFileName ) ) {
        delete fMeshModel;
        fMeshModel = NULL;
        return false;
    }

    // REVISIT - set rotation and translation
//    fMeshModel->SetTranslation( Vec3< float >( -88.0, -99.0, -80.0 ) );
    fMeshModel->SetTranslation( Vec3< float >( -( fMeshModel->GetLB()[ 0 ] + fMeshModel->GetUB()[ 0 ] ) / 2.0,
                                               -( fMeshModel->GetLB()[ 1 ] + fMeshModel->GetUB()[ 1 ] ) / 2.0,
                                               -( fMeshModel->GetLB()[ 2 ] + fMeshModel->GetUB()[ 2 ] ) / 2.0 - 40.0 ) );

    return true;
}

// Initialize cross section plane
bool C3DView::InitializeXsectionPlane( int nSliceIndex,
                                       const QImage &nTexture,
                                       int nWidth,
                                       int nHeight )
{
    // set up intersection plane
	deleteNULL( fPlane );
    fPlane = new CMeshGL();
    fPlane->ComposeVirtualRectangle( 2.0 /* REVISIT */, nWidth, nHeight );	// REVISIT - modify me, FILL ME HERE

    // REVISIT - set rotation and translation
    if ( fMeshModel != NULL ) {
        fPlane->SetTranslation( Vec3< float >(-( fMeshModel->GetLB()[ 0 ] + fMeshModel->GetUB()[ 0 ] ) / 2.0 + nSliceIndex,
                                            -( fMeshModel->GetLB()[ 1 ] + fMeshModel->GetUB()[ 1 ] ) / 2.0,
                                            -( fMeshModel->GetLB()[ 2 ] + fMeshModel->GetUB()[ 2 ] ) / 2.0 - 40.0 ) );
    } else {
        fPlane->SetTranslation( Vec3< float >( -88.0 + nSliceIndex, -99.0, -80.0 ) );
    }
    // REVISIT - http://qt-project.org/doc/qt-4.8/opengl-textures.html
    //           we use QGLWidget::bindTexture() to bind texture, instead of binding it manually
    fPlane->SetTexture( bindTexture( QPixmap::fromImage( nTexture ), GL_TEXTURE_2D ) );

    return true;
}

// Save mesh model
bool C3DView::SaveMeshModel( const std::string &nModelFileName )
{
	return( fMeshModel->SaveModel( nModelFileName ) );
}

// Handle key press event
void C3DView::ProcessKeyEvent( QKeyEvent *nEvent )
{
	switch ( nEvent->key() ) {
	case Qt::Key_W:
		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		updateGL();
		break;
	case Qt::Key_F:
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		updateGL();
		break;
	default:
		break;
	}
}

// Set rotation about x-axis
void C3DView::SetXRotation( int nAngle )
{
	NormalizeAngle( nAngle );
	if ( nAngle != fXRot ) {
		fXRot = nAngle;
		updateGL();
	}
}

// Set rotation about y-axis
void C3DView::SetYRotation( int nAngle )
{
	NormalizeAngle( nAngle );
	if ( nAngle != fYRot ) {
		fYRot = nAngle;
		updateGL();
	}
}

// Set rotation about z-axis
void C3DView::SetZRotation( int nAngle )
{
	NormalizeAngle( nAngle );
	if ( nAngle != fZRot ) {
		fZRot = nAngle;
		updateGL();
	}
}

// Set zoom factor
void C3DView::Zoom( int nDist )
{
    fZoom += nDist;

    // REVISIT - don't write this here
    //           this is basically resizeGL()
    int aW, aH;
    aW = this->geometry().width();
    aH = this->geometry().height();
    int aSide = qMin( aW, aH );

    glViewport( 0, 0, aW, aH );
    float aAspectRatio = float( aW ) / aH;

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();	// REVISIT - missed at 20141230, wrong!
#ifdef QT_OPENGL_ES_1
    glOrthof( -0.5, +0.5, -0.5, +0.5, 4.0, 15.0 );
#else
//    glOrtho ( -0.5, +0.5, -0.5, +0.5, 4.0, 15.0 );
    glOrtho( -fZoom/360.0 * 50.0 * aAspectRatio, +fZoom/360.0 * 50.0 * aAspectRatio, -fZoom/360.0 * 50.0, +fZoom/360.0 * 50.0, -300.0, 300.0 );
#endif // QT_OPENGL_ES_1
    glMatrixMode( GL_MODELVIEW );
}

// Set current slice different so we can translate the slice plane
void C3DView::SetSliceIndexDiff( int nSliceIndexDiff )
{
    // REVISIT - FILL ME HERE
    // REVISIT - we have two ways to translate a model:
    //           1) use glTranslatef() 2) actually change the data, we prefer the first one
    fPlane->ChangeTranslationByDifference( Vec3< float >( nSliceIndexDiff, 0, 0 ) );
    updateGL();
}

// Initialize OpenGL related stuff in order to render the 3D scene
void C3DView::initializeGL( void )
{
    qglClearColor( QColor::fromCmykF( 0.39, 0.39, 0.0, 0.0 ).dark() );

    glEnable( GL_DEPTH_TEST );
//    glEnable( GL_CULL_FACE ); // don't cull faces, visible from both sides
    glEnable( GL_TEXTURE_2D );

//    glShadeModel( GL_SMOOTH );

	// lighting
    glEnable( GL_LIGHTING );
    glEnable( GL_LIGHT0 );
    glEnable( GL_MULTISAMPLE );
    static GLfloat aLightPosition[ 4 ] = { 0.5, 5.0, 7.0, 1.0 };
    glLightfv( GL_LIGHT0, GL_POSITION, aLightPosition );

}

// Resize OpenGL
void C3DView::resizeGL( int nWidth,
                        int nHeight )
{
    int aSide = qMin( nWidth, nHeight );
//    glViewport( ( nWidth - aSide ) / 2, ( nHeight - aSide ) / 2, aSide, aSide );
    glViewport( 0, 0, nWidth, nHeight );
    float aAspectRatio = float( nWidth ) / nHeight;

    glMatrixMode( GL_PROJECTION );
	glLoadIdentity();	// REVISIT - missed at 20141230, wrong!
#ifdef QT_OPENGL_ES_1
    glOrthof( -0.5, +0.5, -0.5, +0.5, 4.0, 15.0 );
#else
//    glOrtho ( -0.5, +0.5, -0.5, +0.5, 4.0, 15.0 );
    glOrtho( -50.0 * aAspectRatio, +50.0 * aAspectRatio, -50.0, +50.0, -300.0, 300.0 );
#endif // QT_OPENGL_ES_1
    glMatrixMode( GL_MODELVIEW );
}

// Paint OpenGL content
// NOTE: For widgets that only need to be decorated with pure OpenGL content, we reimplement QGLWidget::paintGL()
// instead of reimplementing QWidget::paintEvent()
void C3DView::paintGL( void )
{
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    // NOTE: always in GL_MODELVIEW mode
    glLoadIdentity();

    glRotatef( fXRot / 16.0, 1.0, 0.0, 0.0 );
    glRotatef( fYRot / 16.0, 0.0, 1.0, 0.0 );
    glRotatef( fZRot / 16.0, 0.0, 0.0, 1.0 );

//    glTranslatef( -88.0, -99.0, -80.0 ); // position the scene in the center
                                     // REVISIT - FILL ME HERE - hard coded


    if ( fMeshModel != NULL ) {
        fMeshModel->Draw();
    }
    if ( fPlane != NULL ) {
        fPlane->Draw();
    }
}

// Handle mouse press event
void C3DView::mousePressEvent( QMouseEvent *event )
{
    fLastPos = event->pos();
}

// Handle mouse move event
void C3DView::mouseMoveEvent( QMouseEvent *event )
{
    int aDx = event->x() - fLastPos.x();
    int aDy = event->y() - fLastPos.y();

    if ( event->buttons() & Qt::LeftButton ) {
		SetXRotation( fXRot + 8 * aDy );
		SetYRotation( fYRot + 8 * aDx );
    } else if ( event->buttons() & Qt::RightButton ) {
		SetXRotation( fXRot + 8 * aDy );
		SetZRotation( fZRot + 8 * aDx );
    }
    
    // update last postion
    fLastPos = event->pos();
}

// Handle wheel scroll event
void C3DView::wheelEvent( QWheelEvent *event )
{
    // REVISIT - FILL ME HERE
//    QMessageBox::information( this, tr( "wheel event" ), tr( "wheel scrolling detected" ) );
    Zoom( event->delta() / 8 );
    updateGL();
}
