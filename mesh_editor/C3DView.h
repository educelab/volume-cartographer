// C3DView.h
// Chao Du 2014 Dec
#ifndef _C3DVIEW_H_
#define _C3DVIEW_H_

//#include <QtOpenGL> // Qt4
#include <QtOpenGL/QGLWidget> // Qt5


namespace ChaoVis {

class CMeshGL;

class C3DView : public QGLWidget {

    Q_OBJECT

public:
    C3DView( QWidget *parent = 0 );
    ~C3DView( void );

    bool InitializeMeshModel( const std::string &nModelFileName );
    bool InitializeXsectionPlane( int nSliceIndex,
                                  const QImage &nTexture,
                                  int nWidth,
                                  int nHeight );

	bool SaveMeshModel( const std::string &nModelFileName );

    void ProcessKeyEvent( QKeyEvent *nEvent );

	void SetXRotation( int nAngle );
	void SetYRotation( int nAngle );
	void SetZRotation( int nAngle );

    void Zoom( int nDist );

    const CMeshGL* GetMeshModelConst( void ) const { return fMeshModel; }
    CMeshGL* GetMeshModel( void ) { return fMeshModel; }

    void SetSliceIndexDiff( int nSliceIndexDiff );

protected:
    // overwrite OpenGL related functions
    void initializeGL( void );
    void paintGL( void );
    void resizeGL( int nWidth,
                   int nHeight );

    // overwrite mouse handling functions
    void mousePressEvent( QMouseEvent *event );
    void mouseMoveEvent( QMouseEvent *event );
    void wheelEvent( QWheelEvent *event );

private:
    // model
    CMeshGL *fMeshModel;
    CMeshGL *fPlane;

    // rotation angle
    // REVISIT - should be changed to a CTrackBall class
    int fXRot;
    int fYRot;
    int fZRot;
    // REVISIT - refactor and replace the control of the camera with a dedicated trackball
    int fZoom;

    QPoint fLastPos;

}; // class C3DView

} // namespace ChaoVis

#endif // _C3DVIEW_H_
