// CWindow.h
// Chao Du 2014 Dec
#ifndef _CWINDOW_H_
#define _CWINDOW_H_

#include "VCNewGuiHeader.h"

#ifdef _QT5_
#include <QtCore>
#include <QtGui>
#include <QtWidgets>
#else // _QT4_
#include <qt4/Qt/QtGui>
#endif

#include "mathUtils.h"
#include "CBSpline.h"

#include "../ui_VCMain.h"

#include <opencv2/opencv.hpp>

class VolumePkg;

namespace ChaoVis {

class CVolumeViewerWithCurve;

class CWindow : public QMainWindow {

    Q_OBJECT

public:
    enum EWindowtate { WindowStateSegment,      // under segmentation state
                       WindowStateRefine };     // under mesh refinemen state

public:
    CWindow( void );
    ~CWindow( void );

protected:
    void mousePressEvent( QMouseEvent *nEvent );
	void keyPressEvent( QKeyEvent *event );

private:
    void CreateMenus( void );
    void CreateActions( void );

    bool InitializeVolumePkg( const std::string &nVpkgPath );

    void DoSegmentation( void );

private slots:
    void Open( void );
    void Close( void );
    void About( void );

    void OnNewPathClicked( void );

private:
	// data model
    EWindowtate fWindowState;

    VolumePkg   *fVpkg;
    QString     fVpkgPath;
    std::string fVpkgName;

    std::string fSegmentationId;

    int         fPathOnSliceIndex;

    CBSpline    fCurve;

    // REVISIT - how is the point cloud manipulated?
    //           we have this global point cloud, initialized by the starting path, then expanded by segmentation
    //           once we choose editing at a certain slice, we modify the vertices (however, keep the # of vertices unchanged)
    //           and then let the user start segmentation again from this slice. We allow the user to select how many sliced
    //           he want to iterate through, however the newly generated point cloud will overwrite these and the rest data
    //           in the previous point cloud.
//    pcl::PointCloud< pcl::PointXYZRGB > aCurrentCloud;

    // window components
    QMenu		*fFileMenu;
	QMenu		*fEditMenu;
    QMenu       *fHelpMenu;

    QAction		*fOpenVolAct;
    QAction		*fExitAct;
    QAction     *fAboutAct;

    CVolumeViewerWithCurve
                *fVolumeViewerWidget;

    Ui::VCMainWindow    ui;

}; // class CWindow

} // namespace ChaoVis

#endif // _CWINDOW_H_
