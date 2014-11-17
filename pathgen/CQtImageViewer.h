// CQtImageViewer.h
// Chao Du Nov 2014

#ifndef _CQTIMAGEVIEWER_H_
#define _CQTIMAGEVIEWER_H_

#include <QMainWindow>
#include <QPrinter>

#include <opencv2/opencv.hpp>

#include "volumepkg.h"
#include "bezierUtils.h"

class QScrollBar;
class QLabel;
class QScrollArea;

class CQtImageViewer : public QMainWindow
{
	Q_OBJECT

public:
	static cv::Mat QImage2Mat( const QImage  &nSrc );
	static QImage  Mat2QImage( const cv::Mat &nSrc );

public:
	CQtImageViewer( void );
	~CQtImageViewer( void );

	void mousePressEvent( QMouseEvent *nEvent );
	void mouseMoveEvent( QMouseEvent *nEvetn );
	void wheelEvent( QWheelEvent *nWheelEvent );

private slots:
	void Open();
	void Print();
	void ZoomIn();
	void ZoomOut();
	void NormalSize();
	void FitToWindow();
	void About();

    void Close();
    void AboutQt();

private:
	void CreateActions();
	void CreateMenus();
	void UpdateActions();
	void ScaleImage( double nFactor );
	void AdjustScrollBar( QScrollBar *nScrollBar,
							double nFactor );
	double GetScrollPixValue( const QScrollBar *nScrollBar );
	void WidgetLoc2ImgLoc( const cv::Vec2f &nWidgetLoc,
							cv::Vec2f &nImgLoc );


	bool InitializeVolumePkg( const std::string &nVpkgPath );
	void UpdateView( void );

private:
	// Qt GUI
	QLabel *fImageLabel;
	QScrollArea *fScrollArea;
	double fScaleFactor;

#ifndef QT_NO_PRINTER
	QPrinter fPrinter;
#endif

	QAction *fOpenAct;
	QAction *fPrintAct;
	QAction *fExitAct;
	QAction *fZoomInAct;
	QAction *fZoomOutAct;
	QAction *fNormalSizeAct;
	QAction *fFitToWindowAct;
	QAction *fAboutAct;
	QAction *fAboutQtAct;

	QMenu *fFileMenu;
	QMenu *fViewMenu;
    QMenu *fHelpMenu;

	// model
	VolumePkg	*fVpkg;
	QString		fVpkgPath;
	std::string	fVpkgName;
	int			fPathOnSliceIndex;
	cv::Mat		fImgMat;
	cv::Mat		fImgMatCache;
	QImage		fImgQImage;


	int					fTupleIndex;
	pt_tuple			fTmpTuple;
	std::vector< pt_tuple >	fPath;
};

#endif // _CQTIMAGEVIEWER_H_
