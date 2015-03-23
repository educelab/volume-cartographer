// C2DView.h
// Chao Du 2014 Dec
#ifndef _C2DVIEW_H_
#define _C2DVIEW_H_

#include "MeshEditorHeader.h"
#include "CSimpleNumEditBox.h"

#ifdef _QT5_
#include <QtWidgets>
#else // _QT4_
#include <QtGui>
#endif

namespace ChaoVis {

class CXCurve;
class CMeshGL;

class C2DView : public QWidget {

    Q_OBJECT

public:
    C2DView( QWidget *parent = 0 );
    ~C2DView( void );

	void SetImage( const QImage &nSrc );

    void SetIntersection( CXCurve* nCurve );
    void SetIntersection( const std::vector< CXCurve* > &nCurves,
                          int nIndex );
    void SetSliceIndex( int nCurrentSliceIndex );

    void SetMeshModel( CMeshGL *nMeshModel );

protected:
	void mousePressEvent( QMouseEvent *event );
	void mouseMoveEvent( QMouseEvent *event );
	void mouseReleaseEvent( QMouseEvent *event );
	void paintEvent( QPaintEvent *event );
	void resizeEvent( QResizeEvent *event );
    void contextMenuEvent( QContextMenuEvent *event );

private slots:
	void OnZoomInClicked( void );
	void OnZoomOutClicked( void );
	void OnResetClicked( void );
	void OnNextClicked( void );
	void OnPrevClicked( void );
    void OnSliceIndexEditTextChanged( void );

    void SetImpactRange( void );

signals:
	void SendSignalOnNextClicked( void );
	void SendSignalOnPrevClicked( void );
    void SendSignalMeshChanged( void );
    void SendSignalOnUpdateImpactRange( void );
    void SendSignalOnLoadAnySlice( int nSliceIndex );

private:
	void ScaleImage( double nFactor );
	void AdjustScrollBar( QScrollBar *nScrollBar,
						  double nFactor );
	void UpdateButtons( void );
	void DrawCurve( void );
	int SelectPointOnCurve( const CXCurve *nCurve,
							const QPoint &nPt );

    QPoint ConvertWidgetPosToImagePos( const QPoint &nPos );

    double GetScrollPixValue( const QScrollBar *nScrollBar );

private:
	// widget components
	QImage *fImage;
	QLabel *fImageLabel;
	QScrollArea *fScrollArea;
	QPushButton *fZoomInBtn;
	QPushButton *fZoomOutBtn;
	QPushButton *fResetBtn;
	QPushButton *fNextBtn;
    QPushButton *fPrevBtn;
    CSimpleNumEditBox *fSliceIndexEdit;

    QAction *fSetImpactRangeAct;

	// data model
	double fScaleFactor;

    CXCurve *fCurve; // intersection curve, reference to the actual data so we can update curve
    std::vector< CXCurve * > fCurvesLower; // neighboring curves // fCurrentSliceIndex - 1, fCurrentSliceIndex - 2, ...
    std::vector< CXCurve * > fCurvesUpper;                       // fCurrentSliceIndex + 1, fCurrentSliceIndex + 2, ...
	CMeshGL *fMeshModelRef; // reference to the mesh model
    int fCurrentSliceIndex;

	bool fVertexIsChanged;
	int fSelectedPointIndex;

    QPoint fLastPos; // last postion on the image (absolute coordinates)

    int fImpactRange; // how many points a control point movement will affect

}; // class C2DView

} // namespace ChaoVis

#endif // _C2DVIEW_H_
