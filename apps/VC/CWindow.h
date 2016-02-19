// CWindow.h
// Chao Du 2014 Dec
#ifndef _CWINDOW_H_
#define _CWINDOW_H_

#include <QtWidgets>
#include <QRect>
#include <QCloseEvent>
#include <QMessageBox>
#include <opencv2/opencv.hpp>

#include "VCNewGuiHeader.h"
#include "mathUtils.h"
#include "CBSpline.h"
#include "CXCurve.h"
#include "ui_VCMain.h"

#ifndef Q_MOC_RUN
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "structureTensorParticleSim/structureTensorParticleSim.h"
#endif

class VolumePkg;

namespace ChaoVis {

class CVolumeViewerWithCurve;

class CWindow : public QMainWindow {

    Q_OBJECT

public:
    enum EWindowState { WindowStateSegment,     // under segmentation state
                        WindowStateRefine,      // under mesh refinemen state
                        WindowStateDrawPath,    // draw new path
                        WindowStateSegmentation,// segmentation mode
                        WindowStateIdle };      // idle
    enum SaveResponse : bool { Cancelled,
                               Continue};

    typedef struct SSegParams_tag {
        double fGravityScale;
        int fThreshold;
        int fEndOffset;
    } SSegParams;

public:
    CWindow( void );
    CWindow(QRect windowSize);
    ~CWindow( void );

protected:
    void mousePressEvent( QMouseEvent *nEvent );
    void keyPressEvent( QKeyEvent *event );

private:
    void CreateWidgets( void );
    void CreateMenus( void );
    void CreateActions( void );

    void closeEvent(QCloseEvent *closing);

    void setWidgetsEnabled( bool state );

    bool InitializeVolumePkg( const std::string &nVpkgPath );
    SaveResponse SaveDialog( void );

    void UpdateView( void );
    void ChangePathItem( std::string segID );

    void SplitCloud( void );
    void DoSegmentation( void );
    void CleanupSegmentation ( void );
    bool SetUpSegParams( void );

    void SetUpCurves( void );
    void SetCurrentCurve( int nCurrentSliceIndex );

    void OpenSlice( void );

    void InitPathList( void );

    void SetPathPointCloud( void );

    void OpenVolume( void );
    void CloseVolume( void );

    void ResetPointCloud( void );

private slots:
    void Open( void );
    void Close( void );
    void About( void );
    void SavePointCloud( void );

    void OnNewPathClicked( void );
    void OnPathItemClicked( QListWidgetItem* nItem );

    void TogglePenTool( void );
    void ToggleSegmentationTool( void );

    void OnEdtGravityValChange();
    void OnEdtSampleDistValChange( QString nText );
    void OnEdtStartingSliceValChange( QString nText );
    void OnEdtEndingSliceValChange();

    void OnBtnStartSegClicked( void );

    void OnEdtImpactRange( int nImpactRange);

    void OnLoadAnySlice( int nSliceIndex );
    void OnLoadNextSlice( void );
    void OnLoadPrevSlice( void );

    void OnPathChanged( void );

private:
	// data model
    EWindowState fWindowState;

    VolumePkg   *fVpkg;
    QString     fVpkgPath;
    std::string fVpkgName;
    bool        fVpkgChanged;

    std::string fSegmentationId;

    int         fMinSegIndex;
    int         fMaxSegIndex;
    int         fPathOnSliceIndex; // effectively equivalent to the starting slice index

    // for drawing mode
    CBSpline    fSplineCurve; // the curve at current slice
    // for editing mode
    CXCurve     fIntersectionCurve;
    std::vector< CXCurve > fIntersections; // curves of all the slices
//    std::vector< CXCurve > fCurvesLower; // neighboring curves, { -1, -2, ... }
//    std::vector< CXCurve > fCurvesUpper; // neighboring curves, { +1, +2, ... }

    SSegParams  fSegParams;

    // REVISIT - how is the point cloud manipulated?
    //           we have this global point cloud, initialized by the starting path, then expanded by segmentation
    //           once we choose editing at a certain slice, we modify the vertices (however, keep the # of vertices unchanged)
    //           and then let the user start segmentation again from this slice. We allow the user to select how many sliced
    //           he want to iterate through, however the newly generated point cloud will overwrite these and the rest data
    //           in the previous point cloud.
    //           Terminologies:
    //           For our current particle simulation method, we can represent the point cloud with width (# of particles in one slice) and
    //           height (# of iterations). So we can start segmentation from any slice given these two parameters. We call
    //           the point cloud before current slice (3 ~ fPathOnSliceIndex-1) the "upper part", and the newly generated
    //           point cloud from segmentation routine the "lower part". The upper part is from the original point cloud (if there was one)
    //           and the final point cloud is the concatenation of the two parts.
    //           We call the point cloud loaded from disk the "immutable cloud". We only save to disk after the concatenation.
    //           Previously we use a txt file to store vertices of the path where the particle simulation starts. Now they are
    //           stored in "path cloud".
    // REVISIT - maybe redundant
    pcl::PointCloud< pcl::PointXYZRGB > fMasterCloud;  // master cloud, the one and only point cloud
                                                       // can be loaded from disk, or generated from new path
                                                       // or concatenation of upper part and lower part in editing mode
    pcl::PointCloud< pcl::PointXYZRGB > fUpperPart;
    pcl::PointCloud< pcl::PointXYZRGB > fLowerPart;

    // window components
    QMenu       *fFileMenu;
    QMenu       *fHelpMenu;

    QAction     *fOpenVolAct;
    QAction     *fSavePointCloudAct;
    QAction     *fExitAct;
    QAction     *fAboutAct;

    CVolumeViewerWithCurve *fVolumeViewerWidget;
    QListWidget *fPathListWidget;
    QPushButton *fPenTool; // REVISIT - change me to QToolButton
    QPushButton *fSegTool;

    QLineEdit   *fEdtGravity;
    QLineEdit   *fEdtSampleDist;
    QLineEdit   *fEdtStartIndex;
    QLineEdit   *fEdtEndIndex;

    QSlider     *fEdtImpactRange;

    Ui::VCMainWindow    ui;

    QStatusBar  *statusBar;

}; // class CWindow

} // namespace ChaoVis

#endif // _CWINDOW_H_
