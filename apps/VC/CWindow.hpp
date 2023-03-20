#pragma once

#include <QCloseEvent>
#include <QComboBox>
#include <QMessageBox>
#include <QObject>
#include <QRect>
#include <QSpinBox>
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

    using SSegParams = struct SSegParams_tag {
        int fNumIters;
        double fAlpha;
        double fBeta;
        double fDelta;
        double fK1;
        double fK2;
        int fPeakDistanceWeight;
        int fWindowWidth{5};
        bool fIncludeMiddle;
        int targetIndex;
    };

    using Segmenter = volcart::segmentation::LocalResliceSegmentation;

signals:
    void submitSegmentation(Segmenter s);

public slots:
    void onSegmentationFinished(Segmenter::PointSet ps);
    void onSegmentationFailed(std::string s);

public:
    CWindow();
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
    void setDefaultWindowWidth(volcart::Volume::Pointer volume);
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
    void SavePointCloud();

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
    void OnEdtWindowWidthChange(int);
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

    volcart::VolumePkg::Pointer fVpkg;
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

    QSpinBox* fEdtWindowWidth;
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
    size_t progress_{0};
    QLabel* progressLabel_;
    QProgressBar* progressBar_;
};  // class CWindow

class VolPkgBackend : public QObject
{
    Q_OBJECT
public:
    explicit VolPkgBackend(QObject* parent = nullptr) : QObject(parent) {}

signals:
    void segmentationStarted(size_t);
    void segmentationFinished(CWindow::Segmenter::PointSet ps);
    void segmentationFailed(std::string);
    void progressUpdated(size_t);

public slots:
    void startSegmentation(CWindow::Segmenter s)
    {
        segmenter = std::move(s);
        segmenter.progressUpdated.connect(
            [=](size_t p) { progressUpdated(p); });
        segmentationStarted(segmenter.progressIterations());
        try {
            auto result = segmenter.compute();
            segmentationFinished(result);
        } catch (const std::exception& e) {
            segmentationFailed(e.what());
        }
    }

public:
    CWindow::Segmenter segmenter;
};

}  // namespace ChaoVis
