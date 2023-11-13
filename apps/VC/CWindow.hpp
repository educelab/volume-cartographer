#pragma once

#include <QCloseEvent>
#include <QComboBox>
#include <QMessageBox>
#include <QObject>
#include <QRect>
#include <QShortcut>
#include <QSpinBox>
#include <QThread>
#include <QTimer>
#include <QtWidgets>

#include "BlockingDialog.hpp"
#include "CBSpline.hpp"
#include "CXCurve.hpp"
#include "MathUtils.hpp"
#include "ui_VCMain.h"
#include "SegmentationStruct.hpp"

#include "vc/core/types/VolumePkg.hpp"
#include "vc/segmentation/ChainSegmentationAlgorithm.hpp"

#include <thread>
#include <condition_variable>
#include <atomic>
#include <SDL2/SDL.h>
#include <cmath>
#include <queue>
#include <unordered_map>

#define MAX_RECENT_VOLPKG 10

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 6;
static constexpr int VOLPKG_SLICE_MIN_INDEX = 0;

namespace ChaoVis
{

class CVolumeViewerWithCurve;

class AnnotationTreeWidgetItem : public QTreeWidgetItem {
  public:
    AnnotationTreeWidgetItem(QTreeWidget* parent) : QTreeWidgetItem(parent){}

  private:
    bool operator<(const QTreeWidgetItem &other) const {
        int column = treeWidget()->sortColumn();
        return text(column).toInt() < other.text(column).toInt();
    }
};

class CWindow : public QMainWindow
{

    Q_OBJECT

public:
    enum EWindowState {
        WindowStateSegment,       // under segmentation state
        WindowStateRefine,        // under mesh refinement state
        WindowStateDrawPath,      // draw new path
        WindowStateSegmentation,  // segmentation mode
        WindowStateIdle
    };  // idle
    enum SaveResponse : bool { Cancelled, Continue };

    // Structure for segmentation parameters
    // Declare parameters for new algos here and update SetUpSegParams()
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
        int startIndex;
        int targetIndex;
        int targetAnchor{-1};
        // Optical Flow Segmentation Parameters
        bool purge_cache;
        int cache_slices;
        int smoothen_by_brightness;
        int outside_threshold;
        int optical_flow_pixel_threshold;
        int optical_flow_displacement_threshold;
        bool enable_smoothen_outlier;
        bool enable_edge;
        int edge_jump_distance;
        int edge_bounce_distance;
        int smoothness_interpolation_percent;
        int step_size{1};
    };

    using Segmenter = volcart::segmentation::ChainSegmentationAlgorithm;

signals:
    void submitSegmentation(Segmenter::Pointer s);

public slots:
    void onSegmentationFinished(Segmenter::Pointer segmenter, Segmenter::PointSet ps);
    void onSegmentationFailed(Segmenter::Pointer segmenter, std::string s);
    void onShowStatusMessage(QString text, int timeout);
    void onImpactRangeUp(void);
    void onImpactRangeDown(void);

public:
    CWindow();
    ~CWindow(void);

protected:
    void keyPressEvent(QKeyEvent* event);

private:
    void CreateWidgets(void);
    void CreateMenus(void);
    void CreateActions(void);
    void CreateBackend(void);

    void UpdateRecentVolpkgActions(void);
    void UpdateRecentVolpkgList(const QString& path);
    void RemoveEntryFromRecentVolpkg(const QString& path);

    void closeEvent(QCloseEvent* event);

    void setWidgetsEnabled(bool state);

    bool InitializeVolumePkg(const std::string& nVpkgPath);
    void setDefaultWindowWidth(volcart::Volume::Pointer volume);
    SaveResponse SaveDialog(void);

    void UpdateView(void);
    void ChangePathItem(std::string segID);
    void RemovePathItem(std::string segID);

    void SplitCloud(void);
    void DoSegmentation(void);
    void CleanupSegmentation(void);
    bool SetUpSegParams(void);

    void SetUpCurves(void);
    void SetCurrentCurve(int nCurrentSliceIndex);
    void SetUpAnnotations(void);

    void prefetchSlices(void);
    void startPrefetching(int index);
    void OpenSlice(void);

    void InitPathList(void);
    void UpdateAnnotationList(void);

    void SetPathPointCloud(void);
    void ResetPointCloud(void);

    bool prepareSegmentationBaseBefore(std::string algorithm, std::string segID, bool forward, bool anchor, int startIndex, int endIndex);
    void prepareSegmentationBaseAfter(std::string algorithm, std::string segID, bool forward, bool anchor, int endIndex);
    bool prepareSegmentationLRPS(std::string segID, bool forward, bool anchor, int startIndex, int endIndex);
    bool prepareSegmentationOFS(std::string segID, bool forward, bool anchor, int startIndex, int endIndex);
    void queueSegmentation(std::string segmentationId, Segmenter::Pointer s);
    void executeNextSegmentation();

    void OpenVolume(const QString& path);
    void CloseVolume(void);

    static void audio_callback(void *user_data, Uint8 *raw_buffer, int bytes);
    void playPing();

private slots:
    void Open(void);
    void Open(const QString& path);
    void OpenRecent();
    void Close(void);
    void Keybindings(void);
    void About(void);
    void SavePointCloud();
    void ShowSettings();
    void PrintDebugInfo();

    void OnNewPathClicked(void);
    void OnRemovePathClicked(void);
    void OnPathItemClicked(QTreeWidgetItem* item, int column);
    void OnPathItemSelectionChanged();

    void PreviousSelectedId();
    void NextSelectedId();
    void ShowGoToSliceDlg();
    void ScanRangeUp();
    void ScanRangeDown();
    void ReturnToEditSlice();
    void ToggleAnchor();

    void ActivatePenTool();
    void ActivateSegmentationTool();
    void TogglePenTool(void);
    void ToggleSegmentationTool(void);

    void OnChangeSegAlgo(int index);

    void OnEdtAlphaValChange();
    void OnEdtBetaValChange();
    void OnEdtDeltaValChange();
    void OnEdtK1ValChange();
    void OnEdtK2ValChange();
    void OnEdtDistanceWeightChange();
    void OnEdtWindowWidthChange(int);
    void OnOptIncludeMiddleClicked(bool clicked);

    void OnBtnStartSegClicked(void);

    void OnEdtImpactRange(int nImpactRange);

    void OnLoadAnySlice(int slice);
    void OnLoadNextSliceShift(int shift);
    void OnLoadPrevSliceShift(int shift);

    void OnPathChanged(void);
    void OnAnnotationChanged(void);

    void UpdateSegmentCheckboxes(std::string aSegID);
    void toggleDisplayAll(bool checked);
    void toggleComputeAll(bool checked);
    void annotationDoubleClicked(QTreeWidgetItem* item);

    void onBackwardButtonGroupToggled(QAbstractButton* button, bool checked);
    void onForwardButtonGroupToggled(QAbstractButton* button, bool checked);

private:
    // data model
    EWindowState fWindowState;

    volcart::VolumePkg::Pointer fVpkg;
    QString fVpkgPath;
    std::string fVpkgName;
    bool fVpkgChanged;

    std::string fSegmentationId;
    std::string fHighlightedSegmentationId;
    volcart::Volume::Pointer currentVolume;

    static const int AMPLITUDE = 28000;
    static const int FREQUENCY = 44100;

    std::unordered_map<std::string, SegmentationStruct> fSegStructMap;
    int fPathOnSliceIndex; // currently visible slice
    int fSliceIndexToolStart{-1}; // slice for which the currently active tool was started / toggled
    int fEndTargetOffset{5};
    int fFinalTargetIndexForward{-1}; // only for the highlighted segmentation ID
    int fFinalTargetIndexBackward{-1}; // only for the highlighted segmentation ID
    int currentScanRangeIndex{0}; // Index 0 = range size 1 as starting value
    std::vector<int> impactRangeSteps;
    std::vector<int> scanRangeSteps;
    int strideWidth;

    // for drawing mode
    CBSpline fSplineCurve;  // the curve at current slice

    SSegParams fSegParams;
    std::queue<std::pair<std::string, Segmenter::Pointer>> segmentationQueue;
    std::string submittedSegmentationId;

    // window components
    QMenu* fFileMenu;
    QMenu* fViewMenu;
    QMenu* fHelpMenu;
    QMenu* fRecentVolpkgMenu{};

    QAction* fOpenVolAct;
    QAction* fOpenRecentVolpkg[MAX_RECENT_VOLPKG]{};
    QAction* fSavePointCloudAct;
    QAction* fSettingsAct;
    QAction* fExitAct;
    QAction* fKeybinds;
    QAction* fAboutAct;
    QAction* fPrintDebugInfo;

    CVolumeViewerWithCurve* fVolumeViewerWidget;
    QCheckBox* fchkDisplayAll;
    QCheckBox* fchkComputeAll;
    QTreeWidget* fPathListWidget;
    QTreeWidget* fAnnotationListWidget;
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

    // Dynamic OFS algo widgets
    QLabel* lblInterpolationPercent;
    QSpinBox* edtInterpolationPercent;
    QSpinBox* edtStepSize;

    QSlider* fEdtImpactRange;

    // keyboard shortcuts
    QShortcut* slicePrev;
    QShortcut* sliceNext;
    QShortcut* sliceZoomIn;
    QShortcut* sliceZoomOut;
    QShortcut* displayCurves;
    QShortcut* displayCurves_C;
    QShortcut* impactUp;
    QShortcut* impactDwn;
    QShortcut* impactUp_old;
    QShortcut* impactDwn_old;
    QShortcut* segmentationToolShortcut;
    QShortcut* penToolShortcut;
    QShortcut* next1;
    QShortcut* prev1;
    QShortcut* next2;
    QShortcut* prev2;
    QShortcut* next5;
    QShortcut* prev5;
    QShortcut* next10;
    QShortcut* prev10;
    QShortcut* next100;
    QShortcut* prev100;
    QShortcut* prevSelectedId;
    QShortcut* nextSelectedId;
    QShortcut* goToSlice;
    QShortcut* scanRangeUp;
    QShortcut* scanRangeDown;
    QShortcut* returnToEditSlice;
    QShortcut* toggleAnchor;

    Ui_VCMainWindow ui;

    QStatusBar* statusBar;

    bool can_change_volume_();

    QThread worker_thread_;
    BlockingDialog worker_progress_;
    QTimer worker_progress_updater_;
    size_t progress_{0};
    QLabel* progressLabel_;
    QProgressBar* progressBar_;

    // Prefetching worker
    std::thread prefetchWorker;
    std::condition_variable cv;
    std::mutex cv_m;
    std::atomic<bool> stopPrefetching;
    std::atomic<int> prefetchSliceIndex;
};  // class CWindow

class VolPkgBackend : public QObject
{
    Q_OBJECT
public:
    using Segmenter = volcart::segmentation::ChainSegmentationAlgorithm;

    explicit VolPkgBackend(QObject* parent = nullptr) : QObject(parent) {}

signals:
    void segmentationStarted(size_t);
    void segmentationFinished(Segmenter::Pointer, Segmenter::PointSet ps);
    void segmentationFailed(Segmenter::Pointer, std::string);
    void progressUpdated(size_t);

public slots:
    void startSegmentation(Segmenter::Pointer segmenter)
    {
        segmenter->progressUpdated.connect(
            [=](size_t p) { progressUpdated(p); });
        segmentationStarted(segmenter->progressIterations());
        try {
            auto result = segmenter->compute();
            segmentationFinished(segmenter, result);
        } catch (const std::exception& e) {
            segmentationFailed(segmenter, e.what());
        }
    }
};

}  // namespace ChaoVis
