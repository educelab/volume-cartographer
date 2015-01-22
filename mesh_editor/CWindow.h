// CWindow.h
// Chao Du 2014 Dec
#ifndef _CWINDOW_H_
#define _CWINDOW_H_

#include "MeshEditorHeader.h"

#ifdef _QT5_
#include <QtCore>
#include <QtGui>
#include <QtWidgets>
#else // _QT4_
#include <qt4/Qt/QtGui>
#endif

#include "mathUtils.h"


class VolumePkg;

namespace ChaoVis {

class C3DView;
class C2DView;
class CXCurve;
class CMeshGL;

class CWindow : public QMainWindow {//public QWidget {

    Q_OBJECT

public:
    CWindow( void );
    ~CWindow( void );

protected:
	void keyPressEvent( QKeyEvent *event );

private:
    void CreateMenus( void );
    void CreateActions( void );

    // REVISIT - probably we can move volume package related functions to a separate file
    bool InitializeVolumePkg( const std::string &nVpkgPath );
    void OpenSlice( int nSliceIndex );
    void Initialize2DView( void );

	void CalcIntersection( std::vector< CXCurve* >	&nIntersections,
							VolumePkg				*nVpkg,
							const CMeshGL			*nMeshModel );

	void MakeChain( std::vector< Vec2< float > >	&nPts,
					std::vector< int >				&nIndices );

	int FindNearestPoint( std::vector< Vec2< float > >	&nPts,
						const unsigned char				*nPtFlag,
						const Vec2< float >				&nCurPt );

private slots:
    void OpenMesh( void );
    void OpenVol( void );

    void Close( void );

	void GetIntersection( void );

	void OnLoadNextSlice( void );
	void OnLoadPrevSlice( void );

private:
	// data model
    C3DView		*f3DView;		
    C2DView		*f2DView;

    VolumePkg	*fVpkg;
    int			fCurrentSliceIndex;

	QImage		fSliceImage;

	std::vector< CXCurve* > fIntersections;

	// window components
    QMenu		*fFileMenu;
	QMenu		*fEditMenu;

    QAction		*fOpenMeshAct;
    QAction		*fOpenVolAct;
    QAction		*fExitAct;
	QAction		*fGetIntersectionAct;

}; // class CWindow

} // namespace ChaoVis

#endif // _CWINDOW_H_
