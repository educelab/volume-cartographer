#pragma once

#include <QCloseEvent>
#include <QComboBox>
#include <QMessageBox>
#include <QObject>
#include <QRect>
#include <QThread>
#include <QTimer>
#include <QtWidgets>

#include "BlockingDialog.hpp"
#include "CBSpline.hpp"
#include "CXCurve.hpp"
#include "MathUtils.hpp"
#include "ui_VCMain.h"

#include "vc/core/types/VolumePkg.hpp"
#include "vc/segmentation/LocalResliceParticleSim.hpp"

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 6;
static constexpr int VOLPKG_SLICE_MIN_INDEX = 0;

namespace ChaoVis
{

class CVolumeViewerWithCurve;

class CWindow : public QMainWindow
{

    Q_OBJECT

public:
    enum EWindowState {
        WindowStateSegment,       // under segmentation state
        WindowStateRefine,        // under mesh refinemen state
        WindowStateDrawPath,      // draw new path
        WindowStateSegmentation,  // segmentation mode
        WindowStateIdle
    };  // idle
    enum SaveResponse : bool { Cancelled, Continue };

    typedef struct SSegParams_tag {
        int fNumIters;
        double fAlpha;
        double fBeta;
        double fDelta;
        double fK1;
        double fK2;
        int fPeakDistanceWeight;
        int fWindowWidth;
        bool fIncludeMiddle;
        int targetIndex;
    } SSegParams;

    using Segmenter = volcart::segmentation::LocalResliceSegmentation;

signals:
    void submitSegmentation(Segmenter s);

public slots:
    void onSegmentationFinished(Segmenter::PointSet ps);

public:
    CWindow(void);
    CWindow(QRect windowSize);
    ~CWindow(void);

protected:
    void mousePressEvent(QMouseEvent* nEvent);
    void keyPressEvent(QKeyEvent* event);

private:
    void CreateWidgets(void);
    void CreateMenus(void);
    void CreateActions(void);
    void CreateBackend();

    void closeEvent(QCloseEvent* closing);

    void setWidgetsEnabled(bool state);

    bool InitializeVolumePkg(const std::string& nVpkgPath);
    SaveResponse SaveDialog(void);

    void UpdateView(void);
    void ChangePathItem(std::string segID);

    void SplitCloud(void);
    void DoSegmentation(void);
    void CleanupSegmentation(void);
    bool SetUpSegParams(void);

    void SetUpCurves(void);
    void SetCurrentCurve(int nCurrentSliceIndex);

    void OpenSlice(void);

    void InitPathList(void);

    void SetPathPointCloud(void);

    void OpenVolume(void);
    void CloseVolume(void);

    void ResetPointCloud(void);

private slots:
    void Open(void);
    void Close(void);
    void About(void);
    void SavePointCloud(void);

    void OnNewPathClicked(void);
    void OnPathItemClicked(QListWidgetItem* nItem);

    void TogglePenTool(void);
    void ToggleSegmentationTool(void);

    void OnEdtAlphaValChange();
    void OnEdtBetaValChange();
    void OnEdtDeltaValChange();
    void OnEdtK1ValChange();
    void OnEdtK2ValChange();
    void OnEdtDistanceWeightChange();
    void OnEdtWindowWidthChange();
    void OnOptIncludeMiddleClicked(bool clicked);

    // void OnEdtSampleDistValChange( QString nText );
    void OnEdtStartingSliceValChange(QString nText);
    void OnEdtEndingSliceValChange();

    void OnBtnStartSegClicked(void);

    void OnEdtImpactRange(int nImpactRange);

    void OnLoadAnySlice(int nSliceIndex);
    void OnLoadNextSlice(void);
    void OnLoadPrevSlice(void);

    void OnPathChanged(void);

private:
    // data model
    EWindowState fWindowState;

    volcart::VolumePkg* fVpkg;
    QString fVpkgPath;
    std::string fVpkgName;
    bool fVpkgChanged;

    std::string fSegmentationId;
    volcart::Segmentation::Pointer fSegmentation;
    volcart::Volume::Pointer currentVolume;

    int fMinSegIndex;
    int fMaxSegIndex;
    int fPathOnSliceIndex;
    int fEndTargetOffset{5};

    // for drawing mode
    CBSpline fSplineCurve;  // the curve at current slice
    // for editing mode
    CXCurve fIntersectionCurve;
    std::vector<CXCurve> fIntersections;  // curves of all the slices
    //    std::vector< CXCurve > fCurvesLower; // neighboring curves, { -1, -2,
    //    ... }
    //    std::vector< CXCurve > fCurvesUpper; // neighboring curves, { +1, +2,
    //    ... }

    SSegParams fSegParams;

    volcart::OrderedPointSet<cv::Vec3d> fMasterCloud;
    volcart::OrderedPointSet<cv::Vec3d> fUpperPart;
    std::vector<cv::Vec3d> fStartingPath;

    // window components
    QMenu* fFileMenu;
    QMenu* fHelpMenu;

    QAction* fOpenVolAct;
    QAction* fSavePointCloudAct;
    QAction* fExitAct;
    QAction* fAboutAct;

    CVolumeViewerWithCurve* fVolumeViewerWidget;
    QListWidget* fPathListWidget;
    QPushButton* fPenTool;  // REVISIT - change me to QToolButton
    QPushButton* fSegTool;
    QComboBox* volSelect;
    QPushButton* assignVol;

    QLineEdit* fEdtWindowWidth;
    QLineEdit* fEdtDistanceWeight;
    QLineEdit* fEdtAlpha;
    QLineEdit* fEdtBeta;
    QLineEdit* fEdtDelta;
    QLineEdit* fEdtK1;
    QLineEdit* fEdtK2;
    QCheckBox* fOptIncludeMiddle;

    QLineEdit* fEdtStartIndex;
    QLineEdit* fEdtEndIndex;

    QSlider* fEdtImpactRange;
    QLabel* fLabImpactRange;

    Ui_VCMainWindow ui;

    QStatusBar* statusBar;

    bool can_change_volume_();

    QThread worker_thread_;
    BlockingDialog worker_progress_;
    QTimer worker_progress_updater_;
    float progress_{0};
    QLabel* progressLabel_;
    QProgressBar* progressBar_;
};  // class CWindow

class VolPkgBackend : public QObject
{
    Q_OBJECT
public:
    explicit VolPkgBackend(QObject* parent = nullptr) : QObject(parent) {}

signals:
    void segmentationStarted();
    void segmentationFinished(CWindow::Segmenter::PointSet ps);
    void progressUpdated(float);

public slots:
    void startSegmentation(CWindow::Segmenter s)
    {
        segmenter = std::move(s);
        segmenter.progressUpdated.connect([=](float f) { progressUpdated(f); });
        segmentationStarted();
        auto result = segmenter.compute();
        segmentationFinished(result);
    }

public:
    CWindow::Segmenter segmenter;
};

}  // namespace ChaoVis
