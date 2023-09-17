// CWindow.cpp
// Chao Du 2014 Dec
#include "CWindow.hpp"

#include <QKeySequence>
#include <QProgressBar>
#include <QSettings>
#include <opencv2/imgproc.hpp>

#include "CVolumeViewerWithCurve.hpp"
#include "UDataManipulateUtils.hpp"
#include "vc/core/types/Color.hpp"
#include "vc/core/types/Exceptions.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"
#include "vc/segmentation/LocalResliceParticleSim.hpp"
#include "vc/segmentation/OpticalFlowSegmentation.hpp"

namespace vc = volcart;
namespace vcs = volcart::segmentation;
using namespace ChaoVis;
using qga = QGuiApplication;

struct PutTextParams {
    int font{cv::FONT_HERSHEY_SIMPLEX};
    double scale{1};
    int thickness{1};
    int baseline{0};
    cv::Size size;
};

auto CalculateOptimalTextParams(
    const std::string& str,
    int width,
    int height,
    int maxIters = 1000,
    double bufferTB = 0.2,
    double bufferLR = 0.15) -> PutTextParams
{

    // results
    PutTextParams p;

    // calculate the width and height minus the buffer
    auto maxW = width - static_cast<int>(std::ceil(2 * bufferLR * width));
    auto maxH = height - static_cast<int>(std::ceil(2 * bufferTB * height));
    auto minDim = std::min(maxW, maxH);
    auto dIdx = (minDim == maxH) ? 0 : 1;

    // calculate optimal thickness
    const auto x = static_cast<double>(minDim);
    auto t = 9.944e-11 * std::pow(x, 3) + -2.35505e-6 * std::pow(x, 2) +
             1.13691e-2 * x + 0.886545;
    p.thickness = std::min(1, std::max(static_cast<int>(t), 50));

    // iteratively find the correct scale
    for (const auto i : vc::range(maxIters)) {
        p.size =
            cv::getTextSize(str, p.font, p.scale, p.thickness, &p.baseline);
        if (p.size.width >= maxW or p.size.height >= maxH) {
            p.scale *= 0.95;
        } else {
            // get the size dim corresponding to our min dim
            auto minSize = (dIdx == 0) ? p.size.height : p.size.width;
            // scale up if we're great than 10% from our target width
            if (minSize < 0.9 * minDim) {
                p.scale *= 1.11;
            } else {
                break;
            }
        }
    }
    return p;
}

// Constructor
CWindow::CWindow()
    : fWindowState(EWindowState::WindowStateIdle)
    , fVpkg(nullptr)
    , fSegmentationId("")
    , fMinSegIndex(VOLPKG_SLICE_MIN_INDEX)
    , fMaxSegIndex(VOLPKG_SLICE_MIN_INDEX)
    , fPathOnSliceIndex(0)
    , fVolumeViewerWidget(nullptr)
    , fPathListWidget(nullptr)
    , fPenTool(nullptr)
    , fSegTool(nullptr)
    , stopPrefetching(false)
    ,  prefetchSliceIndex(-1)
    , fSegStruct()
{

    ui.setupUi(this);
    ui.splitter->setSizes(QList<int>() << 300 << 100);
    SDL_Init(SDL_INIT_AUDIO);
    fVpkgChanged = false;

    // default parameters for segmentation method
    // REVISIT - refactor me
    fSegParams.fAlpha = 1.0 / 3.0;
    fSegParams.fBeta = 1.0 / 3.0;
    fSegParams.fDelta = 1.0 / 3.0;
    fSegParams.fK1 = 0.5;
    fSegParams.fK2 = 0.5;
    fSegParams.fIncludeMiddle = false;
    fSegParams.fNumIters = 15;
    fSegParams.fPeakDistanceWeight = 50;
    fSegParams.fWindowWidth = 5;
    fSegParams.targetIndex = 5;
    fSegParams.purge_cache = false;
    fSegParams.cache_slices = 300;
    fSegParams.smoothen_by_brightness = 180;
    fSegParams.outside_threshold = 60;
    fSegParams.optical_flow_pixel_threshold = 80;
    fSegParams.optical_flow_displacement_threshold = 10;
    fSegParams.enable_smoothen_outlier = true;
    fSegParams.enable_edge = true;
    fSegParams.edge_jump_distance = 6;
    fSegParams.edge_bounce_distance = 3;
    fSegParams.backwards_smoothnes_interpolation_window = 5;
    fSegParams.backwards_length = 25;

    // create UI widgets
    CreateWidgets();

    // create menu
    CreateActions();
    CreateMenus();
    CreateBackend();

    OpenSlice();
    UpdateView();

    const QSettings settings;
    if (settings.contains("mainWin/geometry")) {
        restoreGeometry(settings.value("mainWin/geometry").toByteArray());
    }
    if (settings.contains("mainWin/state")) {
        restoreState(settings.value("mainWin/state").toByteArray());
    }
}

// Destructor
CWindow::~CWindow(void)
{
    stopPrefetching.store(true);
    cv.notify_one();  // Wake up the thread if it's waitings
    worker_thread_.quit();
    worker_thread_.wait();
    SDL_Quit();
}

// Handle mouse press event
void CWindow::mousePressEvent(QMouseEvent* /*nEvent*/) {}

void CWindow::mouseReleaseEvent(QMouseEvent* /*nEvent*/) {}

// Handle key press event
void CWindow::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Escape) {
        // REVISIT - should prompt warning before exit
        close();
    } else {
        // REVISIT - dispatch key press event
    }
}

// Create widgets
void CWindow::CreateWidgets(void)
{
    // add volume viewer
    QWidget* aTabSegment = this->findChild<QWidget*>("tabSegment");
    assert(aTabSegment != nullptr);

    fVolumeViewerWidget = new CVolumeViewerWithCurve(fSegStructMap);

    QVBoxLayout* aWidgetLayout = new QVBoxLayout;
    aWidgetLayout->addWidget(fVolumeViewerWidget);

    aTabSegment->setLayout(aWidgetLayout);

    // pass the reference of the curve to the widget
    fVolumeViewerWidget->SetSplineCurve(fSplineCurve);
    fVolumeViewerWidget->SetIntersectionCurve(fSegStructMap[fSegmentationId].fIntersectionCurve);

    connect(
        fVolumeViewerWidget, SIGNAL(SendSignalOnNextClicked()), this,
        SLOT(OnLoadNextSlice()));
    connect(
        fVolumeViewerWidget, SIGNAL(SendSignalOnPrevClicked()), this,
        SLOT(OnLoadPrevSlice()));
    connect(
        fVolumeViewerWidget, SIGNAL(SendSignalOnLoadAnyImage(int)), this,
        SLOT(OnLoadAnySlice(int)));
    connect(
        fVolumeViewerWidget, SIGNAL(SendSignalPathChanged()), this,
        SLOT(OnPathChanged()));

    // new path button
    QPushButton* aBtnNewPath = this->findChild<QPushButton*>("btnNewPath");
    QPushButton* aBtnRemovePath =
        this->findChild<QPushButton*>("btnRemovePath");
    aBtnRemovePath->setEnabled(false);
    connect(aBtnNewPath, SIGNAL(clicked()), this, SLOT(OnNewPathClicked()));

    // TODO CHANGE VOLUME LOADING; FIRST CHECK FOR OTHER VOLUMES IN THE STRUCTS
    volSelect = this->findChild<QComboBox*>("volSelect");
    connect(
        volSelect, &QComboBox::currentTextChanged, [this](const QString& text) {
            vc::Volume::Pointer newVolume;
            try {
                newVolume = fVpkg->volume(text.toStdString());
            } catch (const std::out_of_range& e) {
                QMessageBox::warning(this, "Error", "Could not load volume.");
                return;
            }
            currentVolume = newVolume;
            OnLoadAnySlice(0);
            setDefaultWindowWidth(newVolume);
        });

    assignVol = this->findChild<QPushButton*>("assignVol");
    connect(assignVol, &QPushButton::clicked, [this](bool) {
        if (fSegStructMap[fSegmentationId].fSegmentation == nullptr || fSegStructMap[fSegmentationId].fSegmentation->hasVolumeID()) {
            return;
        }
        fSegStructMap[fSegmentationId].fSegmentation->setVolumeID(currentVolume->id());
        UpdateView();
    });

    // pen tool and edit tool
    fPenTool = this->findChild<QPushButton*>("btnPenTool");
    fSegTool = this->findChild<QPushButton*>("btnSegTool");
    connect(fPenTool, SIGNAL(clicked()), this, SLOT(TogglePenTool()));
    connect(fSegTool, SIGNAL(clicked()), this, SLOT(ToggleSegmentationTool()));

    fchkDisplayAll = this->findChild<QCheckBox*>("chkDisplayAll");
    fchkComputeAll = this->findChild<QCheckBox*>("chkComputeAll");
    connect(fchkDisplayAll, &QCheckBox::toggled, this, &CWindow::toggleDisplayAll);
    connect(fchkComputeAll, &QCheckBox::toggled, this, &CWindow::toggleComputeAll);


    // list of paths
    fPathListWidget = this->findChild<QTreeWidget*>("treeWidgetPaths");
    // connect(
    //     fPathListWidget, SIGNAL(itemClicked(QListWidgetItem*)), this,
    //     SLOT(OnPathItemClicked(QListWidgetItem*)));

    connect(fPathListWidget, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(OnPathItemClicked(QTreeWidgetItem*, int)));

    // segmentation methods
    auto* aSegMethodsComboBox = this->findChild<QComboBox*>("cmbSegMethods");
    aSegMethodsComboBox->addItem(tr("Local Reslice Particle Simulation"));
    aSegMethodsComboBox->addItem(tr("Optical Flow Segmentation"));
    connect(
        aSegMethodsComboBox, SIGNAL(currentIndexChanged(int)), this,
        SLOT(OnChangeSegAlgo(int)));

    // ADD NEW SEGMENTATION ALGORITHM NAMES HERE
    // aSegMethodsComboBox->addItem(tr("My custom algorithm"));

    // Optical Flow Segmentation Parameters
    auto* edtOutsideThreshold = new QSpinBox();
    edtOutsideThreshold->setMinimum(0);
    edtOutsideThreshold->setMaximum(255);
    edtOutsideThreshold->setValue(60);
    auto* edtOpticalFlowPixelThreshold = new QSpinBox();
    edtOpticalFlowPixelThreshold->setMinimum(0);
    edtOpticalFlowPixelThreshold->setMaximum(255);
    edtOpticalFlowPixelThreshold->setValue(80);
    auto* edtOpticalFlowDisplacementThreshold = new QSpinBox();
    edtOpticalFlowDisplacementThreshold->setMinimum(0);
    edtOpticalFlowDisplacementThreshold->setValue(10);
    auto* edtSmoothenPixelThreshold = new QSpinBox();
    edtSmoothenPixelThreshold->setMinimum(0);
    edtSmoothenPixelThreshold->setMaximum(256);
    edtSmoothenPixelThreshold->setValue(180);
    auto* chkEnableSmoothenOutlier = new QCheckBox("Smoothen Outlier Points");
    chkEnableSmoothenOutlier->setChecked(true);
    auto* chkEnableEdgeDetection = new QCheckBox("Enable Edge Detection");
    chkEnableEdgeDetection->setChecked(false);
    auto* edtEdgeJumpDistance = new QSpinBox();
    edtEdgeJumpDistance->setMinimum(0);
    edtEdgeJumpDistance->setValue(6);
    auto* edtEdgeBounceDistance = new QSpinBox();
    edtEdgeBounceDistance->setMinimum(0);
    edtEdgeBounceDistance->setValue(3);
    auto* edtBackwardsLength = new QSpinBox();
    edtBackwardsLength->setMinimum(0);
    edtSmoothenPixelThreshold->setMaximum(1000);
    edtBackwardsLength->setValue(25);
    auto* edtBackwardsInterpolationWindow = new QSpinBox();
    edtBackwardsInterpolationWindow->setMinimum(0);
    edtBackwardsInterpolationWindow->setValue(5);
    auto* chkPurgeCache = new QCheckBox("Purge Cache");
    chkPurgeCache->setChecked(false);
    auto* edtCacheSize = new QSpinBox();
    edtCacheSize->setMinimum(-1);
    edtCacheSize->setMaximum(20000);
    edtCacheSize->setValue(300);

    connect(edtOutsideThreshold, &QSpinBox::valueChanged, [=](int v){fSegParams.outside_threshold = v;});
    connect(edtOpticalFlowPixelThreshold, &QSpinBox::valueChanged, [=](int v){fSegParams.optical_flow_pixel_threshold = v;});
    connect(edtOpticalFlowDisplacementThreshold, &QSpinBox::valueChanged, [=](int v){fSegParams.optical_flow_displacement_threshold = v;});
    connect(edtSmoothenPixelThreshold, &QSpinBox::valueChanged, [=](int v){fSegParams.smoothen_by_brightness = v;});
    connect(chkEnableSmoothenOutlier, &QCheckBox::toggled, [=](bool checked){fSegParams.enable_smoothen_outlier = checked;});
    connect(chkEnableEdgeDetection, &QCheckBox::toggled, [=](bool checked){fSegParams.enable_edge = checked;});
    connect(edtEdgeJumpDistance, &QSpinBox::valueChanged, [=](int v){fSegParams.edge_jump_distance = v;});
    connect(edtEdgeBounceDistance, &QSpinBox::valueChanged, [=](int v){fSegParams.edge_bounce_distance = v;});
    connect(edtBackwardsLength, &QSpinBox::valueChanged, [=](int v){fSegParams.backwards_length = v;});
    connect(edtBackwardsInterpolationWindow, &QSpinBox::valueChanged, [=](int v){fSegParams.backwards_smoothnes_interpolation_window = v;});
    connect(chkPurgeCache, &QCheckBox::toggled, [=](bool checked){fSegParams.purge_cache = checked;});
    connect(edtCacheSize, &QSpinBox::valueChanged, [=](int v){fSegParams.cache_slices = v;});

    auto* opticalFlowParamsContainer = new QWidget();
    auto* opticalFlowParamsLayout = new QVBoxLayout(opticalFlowParamsContainer);

    opticalFlowParamsLayout->addWidget(new QLabel("Optical Flow Displacement Threshold"));
    opticalFlowParamsLayout->addWidget(edtOpticalFlowDisplacementThreshold);
    opticalFlowParamsLayout->addWidget(new QLabel("Optical Flow Dark Pixel Threshold"));
    opticalFlowParamsLayout->addWidget(edtOpticalFlowPixelThreshold);
    opticalFlowParamsLayout->addWidget(new QLabel("Smoothen Curve at Dark Points"));
    opticalFlowParamsLayout->addWidget(edtOutsideThreshold);
    opticalFlowParamsLayout->addWidget(new QLabel("Smoothen Curve at Bright Points"));
    opticalFlowParamsLayout->addWidget(edtSmoothenPixelThreshold);
    opticalFlowParamsLayout->addWidget(chkEnableSmoothenOutlier);
    opticalFlowParamsLayout->addWidget(chkEnableEdgeDetection);
    opticalFlowParamsLayout->addWidget(new QLabel("Edge Max Jump Distance"));
    opticalFlowParamsLayout->addWidget(edtEdgeJumpDistance);
    opticalFlowParamsLayout->addWidget(new QLabel("Edge Bounce Distance"));
    opticalFlowParamsLayout->addWidget(edtEdgeBounceDistance);
    opticalFlowParamsLayout->addWidget(new QLabel("Backwards Length"));
    opticalFlowParamsLayout->addWidget(edtBackwardsLength);
    opticalFlowParamsLayout->addWidget(new QLabel("Backwards Interpolation Window"));
    opticalFlowParamsLayout->addWidget(edtBackwardsInterpolationWindow);
    opticalFlowParamsLayout->addWidget(chkPurgeCache);
    opticalFlowParamsLayout->addWidget(new QLabel("Maximum Cache Size"));
    opticalFlowParamsLayout->addWidget(edtCacheSize);

    this->ui.segParamsStack->addWidget(opticalFlowParamsContainer);
    // set the default segmentation method as Optical Flow Segmentation
    aSegMethodsComboBox->setCurrentIndex(1);
    OnChangeSegAlgo(1);

    // LRPS segmentation parameters
    // all of these are contained in this->ui.lrpsParams
    fEdtAlpha = this->findChild<QLineEdit*>("edtAlphaVal");
    fEdtBeta = this->findChild<QLineEdit*>("edtBetaVal");
    fEdtDelta = this->findChild<QLineEdit*>("edtDeltaVal");
    fEdtK1 = this->findChild<QLineEdit*>("edtK1Val");
    fEdtK2 = this->findChild<QLineEdit*>("edtK2Val");
    fEdtDistanceWeight = this->findChild<QLineEdit*>("edtDistanceWeightVal");
    fEdtWindowWidth = this->findChild<QSpinBox*>("edtWindowWidthVal");
    fEdtWindowWidth->setMinimum(3);
    fEdtWindowWidth->setValue(5);
    fOptIncludeMiddle = this->findChild<QCheckBox*>("includeMiddleOpt");
    connect(
        fEdtAlpha, SIGNAL(editingFinished()), this,
        SLOT(OnEdtAlphaValChange()));
    connect(
        fEdtBeta, SIGNAL(editingFinished()), this, SLOT(OnEdtBetaValChange()));
    connect(
        fEdtDelta, SIGNAL(editingFinished()), this,
        SLOT(OnEdtDeltaValChange()));
    connect(fEdtK1, SIGNAL(editingFinished()), this, SLOT(OnEdtK1ValChange()));
    connect(fEdtK2, SIGNAL(editingFinished()), this, SLOT(OnEdtK2ValChange()));
    connect(
        fEdtDistanceWeight, SIGNAL(editingFinished()), this,
        SLOT(OnEdtDistanceWeightChange()));
    connect(
        fEdtWindowWidth, &QSpinBox::valueChanged, this,
        &CWindow::OnEdtWindowWidthChange);
    connect(
        fOptIncludeMiddle, SIGNAL(clicked(bool)), this,
        SLOT(OnOptIncludeMiddleClicked(bool)));

    fEdtStartIndex = this->findChild<QLineEdit*>("edtStartingSliceVal");
    fEdtEndIndex = this->findChild<QLineEdit*>("edtEndingSliceVal");
    connect(
        fEdtStartIndex, SIGNAL(textEdited(QString)), this,
        SLOT(OnEdtStartingSliceValChange(QString)));
    connect(
        fEdtEndIndex, SIGNAL(editingFinished()), this,
        SLOT(OnEdtEndingSliceValChange()));
    

    // INSERT OTHER SEGMENTATION PARAMETER WIDGETS HERE
    // this->ui.segParamsStack->addWidget(new QLabel("Parameter widgets here"));

    // start segmentation button
    QPushButton* aBtnStartSeg = this->findChild<QPushButton*>("btnStartSeg");
    connect(
        aBtnStartSeg, SIGNAL(clicked()), this, SLOT(OnBtnStartSegClicked()));

    // Impact Range slider
    QSlider* fEdtImpactRng = this->findChild<QSlider*>("sldImpactRange");
    connect(
        fEdtImpactRng, SIGNAL(valueChanged(int)), this,
        SLOT(OnEdtImpactRange(int)));
    fLabImpactRange = this->findChild<QLabel*>("labImpactRange");
    fLabImpactRange->setText(QString::number(fEdtImpactRng->value()));

    // Set up the status bar
    statusBar = this->findChild<QStatusBar*>("statusBar");

    // setup shortcuts
    slicePrev = new QShortcut(QKeySequence(tr("Left")), this);
    sliceNext = new QShortcut(QKeySequence(tr("Right")), this);
    sliceZoomIn = new QShortcut(QKeySequence::ZoomIn, this);
    sliceZoomOut = new QShortcut(QKeySequence::ZoomOut, this);
    displayCurves = new QShortcut(QKeySequence(tr(" ")), this);
    displayCurves_C = new QShortcut(QKeySequence(tr("C")), this); // For NoMachine Segmenters
    impactDwn = new QShortcut(QKeySequence(tr("A")), this);
    impactUp = new QShortcut(QKeySequence(tr("D")), this);
    impactDwn_old = new QShortcut(QKeySequence(tr("[")), this);
    impactUp_old = new QShortcut(QKeySequence(tr("]")), this);
    segmentationToolShortcut = new QShortcut(QKeySequence(tr("S")), this);
    penToolShortcut = new QShortcut(QKeySequence(tr("P")), this);
    prev1 = new QShortcut(QKeySequence(tr("1")), this);
    next1 = new QShortcut(QKeySequence(tr("2")), this);
    prev10 = new QShortcut(QKeySequence(tr("3")), this);
    next10 = new QShortcut(QKeySequence(tr("4")), this);
    prev100 = new QShortcut(QKeySequence(tr("5")), this);
    next100 = new QShortcut(QKeySequence(tr("6")), this);


    connect(
        slicePrev, &QShortcut::activated, fVolumeViewerWidget,
        &CVolumeViewerWithCurve::OnPrevClicked);
    connect(
        sliceNext, &QShortcut::activated, fVolumeViewerWidget,
        &CVolumeViewerWithCurve::OnNextClicked);
    connect(
        sliceZoomIn, &QShortcut::activated, fVolumeViewerWidget,
        &CVolumeViewerWithCurve::OnZoomInClicked);
    connect(
        sliceZoomOut, &QShortcut::activated, fVolumeViewerWidget,
        &CVolumeViewerWithCurve::OnZoomOutClicked);
    connect(
        displayCurves, &QShortcut::activated, fVolumeViewerWidget,
        &CVolumeViewerWithCurve::toggleShowCurveBox);
    connect(
        displayCurves_C, &QShortcut::activated, fVolumeViewerWidget,
        &CVolumeViewerWithCurve::toggleShowCurveBox);
    connect(impactUp, &QShortcut::activated, [this]() {
        if (ui.sldImpactRange->isEnabled()) {
            ui.sldImpactRange->triggerAction(
                QSlider::SliderAction::SliderSingleStepAdd);
        }
    });
    connect(impactDwn, &QShortcut::activated, [this]() {
        if (ui.sldImpactRange->isEnabled()) {
            ui.sldImpactRange->triggerAction(
                QSlider::SliderAction::SliderSingleStepSub);
        }
    });
    connect(impactUp_old, &QShortcut::activated, [this]() {
        if (ui.sldImpactRange->isEnabled()) {
            ui.sldImpactRange->triggerAction(
                QSlider::SliderAction::SliderSingleStepAdd);
        }
    });
    connect(segmentationToolShortcut, &QShortcut::activated, this, &CWindow::ActivateSegmentationTool);
    connect(penToolShortcut, &QShortcut::activated, this, &CWindow::ActivatePenTool);
    connect(impactDwn_old, &QShortcut::activated, [this]() {
        if (ui.sldImpactRange->isEnabled()) {
            ui.sldImpactRange->triggerAction(
                QSlider::SliderAction::SliderSingleStepSub);
        }
    });
    connect(next1, &QShortcut::activated, [this]() {
        int shift = 1;
        OnLoadNextSliceShift(shift);
    });
    connect(prev1, &QShortcut::activated, [this]() {
        int shift = 1;
        OnLoadPrevSliceShift(shift);
    });
    connect(next10, &QShortcut::activated, [this]() {
        int shift = 10;
        OnLoadNextSliceShift(shift);
    });
    connect(prev10, &QShortcut::activated, [this]() {
        int shift = 10;
        OnLoadPrevSliceShift(shift);
    });
    connect(next100, &QShortcut::activated, [this]() {
        int shift = 100;
        OnLoadNextSliceShift(shift);
    });
    connect(prev100, &QShortcut::activated, [this]() {
        int shift = 100;
        OnLoadPrevSliceShift(shift);
    });
}

// Create menus
void CWindow::CreateMenus(void)
{
    fFileMenu = new QMenu(tr("&File"), this);
    fFileMenu->addAction(fOpenVolAct);
    fFileMenu->addAction(fSavePointCloudAct);
    fFileMenu->addSeparator();
    fFileMenu->addAction(fExitAct);

    fHelpMenu = new QMenu(tr("&Help"), this);
    fHelpMenu->addAction(fKeybinds);
    fHelpMenu->addAction(fAboutAct);

    menuBar()->addMenu(fFileMenu);
    menuBar()->addMenu(fHelpMenu);
}

// Create actions
void CWindow::CreateActions(void)
{
    fOpenVolAct = new QAction(tr("&Open volpkg..."), this);
    connect(fOpenVolAct, SIGNAL(triggered()), this, SLOT(Open()));
    fOpenVolAct->setShortcut(QKeySequence::Open);

    fExitAct = new QAction(tr("E&xit..."), this);
    connect(fExitAct, SIGNAL(triggered()), this, SLOT(Close()));

    fKeybinds = new QAction(tr("&Keybinds"), this);
    connect(fKeybinds, SIGNAL(triggered()), this, SLOT(Keybindings()));

    fAboutAct = new QAction(tr("&About..."), this);
    connect(fAboutAct, SIGNAL(triggered()), this, SLOT(About()));

    fSavePointCloudAct = new QAction(tr("&Save volpkg..."), this);
    connect(
        fSavePointCloudAct, SIGNAL(triggered()), this, SLOT(SavePointCloud()));
    fSavePointCloudAct->setShortcut(QKeySequence::Save);
}

void CWindow::CreateBackend()
{
    // Setup backend runner
    auto* worker = new VolPkgBackend();
    worker->moveToThread(&worker_thread_);
    connect(&worker_thread_, &QThread::finished, worker, &QObject::deleteLater);
    connect(
        this, &CWindow::submitSegmentation, worker,
        &VolPkgBackend::startSegmentation);
    connect(
        worker, &VolPkgBackend::segmentationFinished, this,
        &CWindow::onSegmentationFinished);
    connect(
        worker, &VolPkgBackend::segmentationFailed, this,
        &CWindow::onSegmentationFailed);
    connect(worker, &VolPkgBackend::progressUpdated, [=](size_t p) {
        progress_ = p;
    });
    worker_thread_.start();

    // Setup progress dialog
    auto layout = new QVBoxLayout();
    worker_progress_.setLayout(layout);
    progressLabel_ = new QLabel("Segmentation in progress. Please wait...");
    layout->addWidget(progressLabel_);
    progressBar_ = new QProgressBar();
    layout->addWidget(progressBar_);
    progressBar_->setMinimum(0);
    connect(worker, &VolPkgBackend::segmentationStarted, [=](size_t its) {
        progressBar_->setMaximum(its);
    });

    // Update the GUI intermittently
    worker_progress_updater_.setInterval(1000);
    connect(&worker_progress_updater_, &QTimer::timeout, [=]() {
        if (progressLabel_->text() ==
            "Segmentation in progress. Please wait...") {
            progressLabel_->setText("Segmentation in progress. Please wait");
        } else {
            progressLabel_->setText(progressLabel_->text().append('.'));
        }
        progressBar_->setValue(progress_);
    });
}

// Asks User to Save Data Prior to VC.app Exit
void CWindow::closeEvent(QCloseEvent* closing)
{
    if (SaveDialog() == SaveResponse::Continue) {
        closing->accept();
    } else {
        closing->ignore();
    }
    QSettings settings;
    settings.setValue("mainWin/geometry", saveGeometry());
    settings.setValue("mainWin/state", saveState());
}

void CWindow::setWidgetsEnabled(bool state)
{
    this->findChild<QGroupBox*>("grpVolManager")->setEnabled(state);
    this->findChild<QGroupBox*>("grpSeg")->setEnabled(state);
    this->findChild<QPushButton*>("btnSegTool")->setEnabled(state);
    this->findChild<QPushButton*>("btnPenTool")->setEnabled(state);
    this->findChild<QGroupBox*>("groupBox_4")->setEnabled(state);
    fVolumeViewerWidget->setButtonsEnabled(state);
}

bool CWindow::InitializeVolumePkg(const std::string& nVpkgPath)
{
    fVpkg = nullptr;

    try {
        fVpkg = vc::VolumePkg::New(nVpkgPath);
    } catch (const std::exception& e) {
        vc::Logger()->error("Failed to initialize volpkg: {}", e.what());
    }

    fVpkgChanged = false;

    if (fVpkg == nullptr) {
        vc::Logger()->error("Cannot open .volpkg: {}", nVpkgPath);
        QMessageBox::warning(
            this, "Error",
            "Volume package failed to load. Package might be corrupt.");
        return false;
    }
    return true;
}

void CWindow::setDefaultWindowWidth(vc::Volume::Pointer volume)
{
    // Update window width based on selected volume
    auto winWidth = std::ceil(fVpkg->materialThickness() / volume->voxelSize());
    fEdtWindowWidth->setValue(static_cast<int>(winWidth));
}

CWindow::SaveResponse CWindow::SaveDialog(void)
{
    // Return if nothing has changed
    if (not fVpkgChanged) {
        return SaveResponse::Continue;
    }

    const auto response = QMessageBox::question(
        this, "Save changes?",
        tr("Changes will be lost! Save volume package before continuing?\n"),
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    switch (response) {
        case QMessageBox::Save:
            SavePointCloud();
            return SaveResponse::Continue;
        case QMessageBox::Discard:
            fVpkgChanged = false;
            return SaveResponse::Continue;
        default:
            return SaveResponse::Cancelled;
    }
    return SaveResponse::Cancelled;
}

// Update the widgets
void CWindow::UpdateView(void)
{
    if (fVpkg == nullptr) {
        setWidgetsEnabled(false);  // Disable Widgets for User
        this->findChild<QLabel*>("lblVpkgName")
            ->setText("[ No Volume Package Loaded ]");
        return;
    }

    setWidgetsEnabled(true);  // Enable Widgets for User

    // show volume package name
    this->findChild<QLabel*>("lblVpkgName")
        ->setText(QString(fVpkg->name().c_str()));

    // set widget accessibility properly based on the states: is drawing? is
    // editing?
    fEdtAlpha->setText(QString("%1").arg(fSegParams.fAlpha));
    fEdtBeta->setText(QString("%1").arg(fSegParams.fBeta));
    fEdtDelta->setText(QString("%1").arg(fSegParams.fDelta));
    fEdtK1->setText(QString("%1").arg(fSegParams.fK1));
    fEdtK2->setText(QString("%1").arg(fSegParams.fK2));
    fEdtDistanceWeight->setText(
        QString("%1").arg(fSegParams.fPeakDistanceWeight));
    fEdtWindowWidth->setValue(fSegParams.fWindowWidth);
    fEdtStartIndex->setText(QString("%1").arg(fPathOnSliceIndex));

    if (fPathOnSliceIndex + fEndTargetOffset >= currentVolume->numSlices()) {
        fEdtEndIndex->setText(QString::number(currentVolume->numSlices() - 1));
    } 
    else if (fPathOnSliceIndex + fEndTargetOffset < 0) {
        fEdtEndIndex->setText(QString::number(0));
    }    
    else {
        fEdtEndIndex->setText(
            QString::number(fPathOnSliceIndex + fEndTargetOffset));
    }

    // Logic to enable/disable segmentation and pen tools. TODO add logic to check propper segmentations
    bool availableSegments = false;
    bool availableNewSegments = false;
    for (auto& seg : fSegStructMap) {
        auto& segStruct = seg.second;
        if (!segStruct.display && !segStruct.compute) {
            continue;
        }
        // segments with already existing line
        availableSegments = availableSegments || segStruct.fIntersectionCurve.GetPointsNum() > 0;
        // completely empty segments, for drawing curve
        availableNewSegments = availableNewSegments || (!segStruct.fSegmentationId.empty() && segStruct.fMasterCloud.empty());
    }
    fSegTool->setEnabled(!availableNewSegments && availableSegments);
    fPenTool->setEnabled(availableNewSegments);

    volSelect->setEnabled(can_change_volume_());
    assignVol->setEnabled(can_change_volume_());

    // REVISIT - these two states should be mutually exclusive, we guarantee
    // this when we toggle the button, BUGGY!
    if (fWindowState == EWindowState::WindowStateIdle) {
        fVolumeViewerWidget->SetViewState(
            CVolumeViewerWithCurve::EViewState::ViewStateIdle);
        this->findChild<QGroupBox*>("grpVolManager")->setEnabled(true);
        this->findChild<QGroupBox*>("grpSeg")->setEnabled(false);
    } else if (fWindowState == EWindowState::WindowStateDrawPath) {
        fVolumeViewerWidget->SetViewState(
            CVolumeViewerWithCurve::EViewState::ViewStateDraw);
        this->findChild<QGroupBox*>("grpVolManager")->setEnabled(false);
        this->findChild<QGroupBox*>("grpSeg")->setEnabled(false);
    } else if (fWindowState == EWindowState::WindowStateSegmentation) {
        fVolumeViewerWidget->SetViewState(
            CVolumeViewerWithCurve::EViewState::ViewStateEdit);
        this->findChild<QGroupBox*>("grpVolManager")->setEnabled(false);
        this->findChild<QGroupBox*>("grpSeg")->setEnabled(true);
    } else {
        // something else
    }

    fEdtStartIndex->setEnabled(false);

    fVolumeViewerWidget->UpdateView();

    update();
}

// Reset point cloud
void CWindow::ResetPointCloud(void)
{
    for (auto& seg : fSegStructMap) {
        auto& segStruct = seg.second;
        segStruct.ResetPointCloud();
    }
}

// Activate a specific segmentation by ID
void CWindow::ChangePathItem(std::string segID)
{
    statusBar->clearMessage();
    fSegmentationId = segID;

    // write new Segment to fSegStructMap
    fSegStructMap[fSegmentationId] = SegmentationStruct(fVpkg, segID, fPathOnSliceIndex);

    if (fSegStructMap[fSegmentationId].currentVolume != nullptr && fSegStructMap[fSegmentationId].fSegmentation->hasVolumeID()) {
        currentVolume = fSegStructMap[fSegmentationId].currentVolume;
    }

    // Only change slices if no other segmentations are being displayed
    bool setPathIndex = true;
    for (auto& seg : fSegStructMap) {
        auto& segStruct = seg.second;
        if (segStruct.display || segStruct.compute) {
            setPathIndex = false;
            break;
        }
    }
    if (setPathIndex && !fSegStructMap[fSegmentationId].fMasterCloud.empty()) {
        fPathOnSliceIndex = fSegStructMap[fSegmentationId].fPathOnSliceIndex;
    }

    OpenSlice();
    SetCurrentCurve(fPathOnSliceIndex);
    UpdateView();
}

// Deactivate a specific segmentation by ID. TODO: finish implementation?
void CWindow::RemovePathItem(std::string segID)
{
    statusBar->clearMessage();
}

// Split fMasterCloud into fUpperCloud and fLowerCloud
void CWindow::SplitCloud(void)
{
    for (auto& seg : fSegStructMap) {
        auto& segStruct = seg.second;
        segStruct.SetPathOnSliceIndex(fPathOnSliceIndex);
        segStruct.SplitCloud();
    }
}

// Do segmentation given the starting point cloud
void CWindow::DoSegmentation(void)
{
    statusBar->clearMessage();

    // Make sure our seg params structure has the current values
    if (not SetUpSegParams()) {
        QMessageBox::information(
            this, tr("Info"), tr("Invalid parameter for segmentation"));
        return;
    }

    // Setup LRPS
    auto segIdx = this->ui.cmbSegMethods->currentIndex();
    // Reminder to activate the segments for computation
    bool segmentedSomething = false;
    for (auto& seg : fSegStructMap) {
        auto& segStruct = seg.second;
        auto& segID = seg.first;

        // qDebug() << "Segment " << segID.c_str() << " display: " << segStruct.display << " compute: " << segStruct.compute;

        // if the segmentation is not being computed, skip it
        if (!segStruct.display || !segStruct.compute) {
            continue;
        }

        // if the segment is not on the starting index, skip it
        if (segStruct.fStartingPath.empty()) {
            // qDebug() << "Segment " << segID.c_str() << " has no starting path!";
            continue;
        }

        segmentedSomething = true;

        Segmenter::Pointer segmenter;
        if (segIdx == 0) {
            auto lrps = vcs::LocalResliceSegmentation::New();
            lrps->setMaterialThickness(fVpkg->materialThickness());
            lrps->setTargetZIndex(fSegParams.targetIndex);
            lrps->setOptimizationIterations(fSegParams.fNumIters);
            lrps->setResliceSize(fSegParams.fWindowWidth);
            lrps->setAlpha(fSegParams.fAlpha);
            lrps->setK1(fSegParams.fK1);
            lrps->setK2(fSegParams.fK2);
            lrps->setBeta(fSegParams.fBeta);
            lrps->setDelta(fSegParams.fDelta);
            lrps->setDistanceWeightFactor(fSegParams.fPeakDistanceWeight);
            lrps->setConsiderPrevious(fSegParams.fIncludeMiddle);
            segmenter = lrps;
        }
        if (segIdx == 1) {
            auto ofsc = vcs::OpticalFlowSegmentationClass::New();
            ofsc->setMaterialThickness(fVpkg->materialThickness());
            ofsc->setTargetZIndex(fSegParams.targetIndex);
            ofsc->setOptimizationIterations(fSegParams.fNumIters);
            ofsc->setOutsideThreshold(fSegParams.outside_threshold);
            ofsc->setOFThreshold(fSegParams.optical_flow_pixel_threshold);
            ofsc->setOFDispThreshold(fSegParams.optical_flow_displacement_threshold);
            ofsc->setLineSmoothenByBrightness(fSegParams.smoothen_by_brightness);
            ofsc->setEdgeJumpDistance(fSegParams.edge_jump_distance);
            ofsc->setEdgeBounceDistance(fSegParams.edge_bounce_distance);
            ofsc->setEnableSmoothenOutlier(fSegParams.enable_smoothen_outlier);
            ofsc->setEnableEdge(fSegParams.enable_edge);
            ofsc->setPurgeCache(fSegParams.purge_cache);
            ofsc->setCacheSlices(fSegParams.cache_slices);
            ofsc->setOrderedPointSet(fSegStructMap[segID].fMasterCloud);
            ofsc->setBackwardsInterpolationWindow(fSegParams.backwards_smoothnes_interpolation_window);
            ofsc->setBackwardsLength(fSegParams.backwards_length);
            segmenter = ofsc;
        }
        // ADD OTHER SEGMENTER SETUP HERE. MATCH THE IDX TO THE IDX IN THE
        // DROPDOWN LIST

        // set common parameters
        segmenter->setChain(fSegStructMap[segID].fStartingPath);
        segmenter->setVolume(currentVolume);
        // Que Segmentation for execution
        queueSegmentation(segID, segmenter);
    }

    if (!segmentedSomething) {
        QMessageBox::warning(
            this, "Warning", "No Segments for computation found! Please activate segments for computation in the segment manager and make sure to be on a slice containing at least one curve.");
        segmentationQueue = std::queue<std::pair<std::string, Segmenter::Pointer>>();
    }

    // Start
    executeNextSegmentation();
}

void CWindow::queueSegmentation(std::string segmentationId, Segmenter::Pointer s)
{
    segmentationQueue.push(std::make_pair(segmentationId, s));
}

void CWindow::executeNextSegmentation()
{
    if (!segmentationQueue.empty()) {
        auto[segmentId, nextSegmenter]  = segmentationQueue.front();
        submittedSegmentationId = segmentId;
        segmentationQueue.pop();
        submitSegmentation(nextSegmenter);
        setWidgetsEnabled(false);
        worker_progress_.show();
        worker_progress_updater_.start();
    }
    else {
        setWidgetsEnabled(true);
        // set display to target layer
        fPathOnSliceIndex = fSegParams.targetIndex;
        CleanupSegmentation();
        SetUpCurves();
        UpdateView();
        playPing();
    }
}

void CWindow::audio_callback(void *user_data, Uint8 *raw_buffer, int bytes) {
        Sint16 *buffer = reinterpret_cast<Sint16*>(raw_buffer);
        int length = bytes / 2; // 2 bytes per sample for AUDIO_S16SYS
        int &sample_nr = *reinterpret_cast<int*>(user_data);

        for(int i = 0; i < length; i++, sample_nr++)
        {
            double time = static_cast<double>(sample_nr) / FREQUENCY;
            // This will give us a sine wave at 440 Hz
            buffer[i] = static_cast<Sint16>(AMPLITUDE * std::sin(2.0f * 3.14159f * 440.0f * time));
        }
    }

void CWindow::playPing() {
    SDL_AudioSpec desiredSpec;

    desiredSpec.freq = FREQUENCY;
    desiredSpec.format = AUDIO_S16SYS;
    desiredSpec.channels = 0;
    desiredSpec.samples = 2048;
    desiredSpec.callback = audio_callback;

    int sample_nr = 0;

    desiredSpec.userdata = &sample_nr;

    SDL_AudioSpec obtainedSpec;

    // you might want to look for errors here
    SDL_OpenAudio(&desiredSpec, &obtainedSpec);

    // start play audio
    SDL_PauseAudio(0);

    // play for 1000 milliseconds (1.0 second)
    SDL_Delay(1000);

    // Stop audio playback
    SDL_PauseAudio(1);

    SDL_CloseAudio();
}

void CWindow::onSegmentationFinished(Segmenter::PointSet ps)
{
    worker_progress_updater_.stop();
    worker_progress_.close();
    // 3) concatenate the two parts to form the complete point cloud
    // find starting location in fMasterCloud
    int i;
    for (i= 0; i < fSegStructMap[submittedSegmentationId].fMasterCloud.height(); i++) {
        auto masterRowI = fSegStructMap[submittedSegmentationId].fMasterCloud.getRow(i);
        if (ps[0][2] <= masterRowI[fSegStructMap[submittedSegmentationId].fUpperPart.width()-1][2]){
            break;
        }
    }

    // remove the duplicated point and ps in their stead. if i at the end, no duplicated point, just append
    fSegStructMap[submittedSegmentationId].fUpperPart = fSegStructMap[submittedSegmentationId].fMasterCloud.copyRows(0, i);
    fSegStructMap[submittedSegmentationId].fUpperPart.append(ps);

    // check if remaining rows already exist in fMasterCloud behind ps
    for(; i < fSegStructMap[submittedSegmentationId].fMasterCloud.height(); i++) {
        auto masterRowI = fSegStructMap[submittedSegmentationId].fMasterCloud.getRow(i);
        if (ps[ps.size() - 1][2] < masterRowI[fSegStructMap[submittedSegmentationId].fUpperPart.width()-1][2]) {
            break;
        }
    }
    // add the remaining rows
    if (i < fSegStructMap[submittedSegmentationId].fMasterCloud.height()) {
        fSegStructMap[submittedSegmentationId].fUpperPart.append(fSegStructMap[submittedSegmentationId].fMasterCloud.copyRows(i, fSegStructMap[submittedSegmentationId].fMasterCloud.height()));
    }

    fSegStructMap[submittedSegmentationId].fMasterCloud = fSegStructMap[submittedSegmentationId].fUpperPart;

    // qDebug() << "Segmentation finished: " << submittedSegmentationId.c_str();
    // for (int u = 0; u < fSegStructMap[submittedSegmentationId].fMasterCloud.height(); u++) {
    //     auto masterRowI = fSegStructMap[submittedSegmentationId].fMasterCloud.getRow(u);
    //     qDebug() << "Row " << u << " has " << masterRowI.size() << " points. With z: " << masterRowI[fSegStructMap[submittedSegmentationId].fUpperPart.width()-1][2];
    // }

    statusBar->showMessage(tr("Segmentation complete"));
    fVpkgChanged = true;
    
    // Execute the next segmentation
    executeNextSegmentation();
}

void CWindow::onSegmentationFailed(std::string s)
{
    vc::Logger()->error("Segmentation failed: {}", s);
    statusBar->showMessage(tr("Segmentation failed"));
    QMessageBox::critical(
        this, tr("VC"), QString::fromStdString("Segmentation failed:\n\n" + s));

    // Execute the next segmentation
    executeNextSegmentation();

    // setWidgetsEnabled(true);
    // worker_progress_updater_.stop();
    // worker_progress_.close();
    // CleanupSegmentation();
    // UpdateView();
}

void CWindow::CleanupSegmentation(void)
{
    fSegTool->setChecked(false);
    fWindowState = EWindowState::WindowStateIdle;
    SetUpCurves();
    OpenSlice();
    SetCurrentCurve(fPathOnSliceIndex);
}

// Set up the parameters for doing segmentation
bool CWindow::SetUpSegParams(void)
{
    bool aIsOk;

    double alpha = fEdtAlpha->text().toDouble(&aIsOk);
    if (aIsOk) {
        fSegParams.fAlpha = alpha;
    } else {
        return false;
    }

    double beta = fEdtBeta->text().toDouble(&aIsOk);
    if (aIsOk) {
        fSegParams.fBeta = beta;
    } else {
        return false;
    }

    double delta = fEdtDelta->text().toDouble(&aIsOk);
    if (aIsOk) {
        fSegParams.fDelta = delta;
    } else {
        return false;
    }

    double k1 = fEdtK1->text().toDouble(&aIsOk);
    if (aIsOk) {
        fSegParams.fK1 = k1;
    } else {
        return false;
    }

    double k2 = fEdtK2->text().toDouble(&aIsOk);
    if (aIsOk) {
        fSegParams.fK2 = k2;
    } else {
        return false;
    }

    int aNewVal = fEdtDistanceWeight->text().toInt(&aIsOk);
    if (aIsOk) {
        fSegParams.fPeakDistanceWeight = aNewVal;
    } else {
        return false;
    }

    fSegParams.fWindowWidth = fEdtWindowWidth->value();

    fSegParams.fIncludeMiddle = fOptIncludeMiddle->isChecked();

    // ending slice index
    aNewVal = fEdtEndIndex->text().toInt(&aIsOk);
    if (aIsOk &&
        aNewVal < currentVolume->numSlices()) {
        fSegParams.targetIndex = aNewVal;
    } else {
        return false;
    }

    return true;
}

// Get the curves for all the slices
void CWindow::SetUpCurves(void)
{
    // if (fVpkg == nullptr || fSegStructMap[fSegmentationId].fMasterCloud.empty()) {
    //     statusBar->showMessage(tr("Selected point cloud is empty"));
    //     vc::Logger()->warn("Segmentation point cloud is empty");
    //     return;
    // }
    for (auto& seg : fSegStructMap) {
        auto& segStruct = seg.second;
        segStruct.SetUpCurves();
    }
}

// Set the current curve
void CWindow::SetCurrentCurve(int nCurrentSliceIndex)
{
    // qDebug() << "SetCurrentCurve: " << nCurrentSliceIndex;
    for (auto& seg : fSegStructMap) {
        auto& segStruct = seg.second;
        segStruct.SetCurrentCurve(nCurrentSliceIndex);
    }
}

void CWindow::prefetchSlices(void) {
  while (true) {
    std::unique_lock<std::mutex> lk(cv_m);
    cv.wait(lk, [this]{return prefetchSliceIndex != -1;});

    if (stopPrefetching.load()) {
      break;
    }

    int prefetchWindow = 100;
    int currentSliceIndex = prefetchSliceIndex.load();
    int start = std::max(0, currentSliceIndex - prefetchWindow);
    int end = std::min(currentVolume->numSlices()-1, currentSliceIndex + prefetchWindow);

    int n = 5;  // Number Fetching Threads
    // fetching from index outwards
    for (int offset = 0; offset <= prefetchWindow; offset = offset + n) {
        std::vector<std::thread> threads;

        for (int i = 0; i <= n; i++) {
            // Fetch the slice data on the right side
            // Fetch the slice data on the right side
            if (currentSliceIndex + offset + i <= end) {
                threads.emplace_back(&volcart::Volume::getSliceData, currentVolume, currentSliceIndex + offset + i);
            }
            // Fetch the slice data on the left side
            if (currentSliceIndex - offset - i >= start) {
                threads.emplace_back(&volcart::Volume::getSliceData, currentVolume, currentSliceIndex - offset - i);
            }
        }

        for (auto& t : threads) {
            t.join();
        }

        // Check if prefetching was stopped or slice index changed
        if (stopPrefetching.load() || prefetchSliceIndex.load() != currentSliceIndex) {
            break;
        }
    }

    prefetchSliceIndex = -1;
  }
}

// Function to start prefetching around a certain slice
void CWindow::startPrefetching(int index) {
  prefetchSliceIndex = index;
  cv.notify_one();
}

// Open slice
void CWindow::OpenSlice(void)
{
    cv::Mat aImgMat;
    if (fVpkg != nullptr) {
        // Stop prefetching
        prefetchSliceIndex = -1;
        cv.notify_one();

        aImgMat = currentVolume->getSliceDataCopy(fPathOnSliceIndex);
        aImgMat.convertTo(aImgMat, CV_8UC1, 1.0 / 256.0);
        //        cvtColor(aImgMat, aImgMat, cv::COLOR_GRAY2BGR);
    } else {
        aImgMat = cv::Mat::zeros(10, 10, CV_8UC1);
    }

    if (aImgMat.empty()) {
        auto h = currentVolume->sliceHeight();
        auto w = currentVolume->sliceWidth();
        aImgMat = cv::Mat::zeros(h, w, CV_8UC3);
        aImgMat = vc::color::RED;
        const std::string msg{"FILE MISSING"};
        auto params = CalculateOptimalTextParams(msg, w, h);
        auto originX = (w - params.size.width) / 2;
        auto originY = params.size.height + (h - params.size.height) / 2;
        cv::Point origin{originX, originY};
        cv::putText(
            aImgMat, msg, origin, params.font, params.scale, vc::color::WHITE,
            params.thickness, params.baseline);
    }

    auto aImgQImage = Mat2QImage(aImgMat);
    fVolumeViewerWidget->SetImage(aImgQImage);
    fVolumeViewerWidget->SetImageIndex(fPathOnSliceIndex);
}

// Initialize path list
void CWindow::InitPathList(void)
{
    fPathListWidget->clear();
    if (fVpkg != nullptr) {
        // show the existing paths
        for (auto& s : fVpkg->segmentationIDs()) {
            QTreeWidgetItem *item = new QTreeWidgetItem(fPathListWidget);
            item->setText(0, QString(s.c_str()));
            item->setCheckState(1, Qt::Unchecked);
            item->setCheckState(2, Qt::Unchecked);
        }
    }
}

// Update the Master cloud with the path we drew
void CWindow::SetPathPointCloud(void)
{
    // calculate the path and save that to aMasterCloud
    std::vector<cv::Vec2f> aSamplePts;
    fSplineCurve.GetSamplePoints(aSamplePts);

    // remove duplicates
    auto numPts = aSamplePts.size();
    auto unique = std::unique(aSamplePts.begin(), aSamplePts.end());
    aSamplePts.erase(unique, aSamplePts.end());
    auto uniquePts = aSamplePts.size();
    vc::Logger()->warn("Removed {} duplicate points", numPts - uniquePts);

    // setup a new master cloud
    fSegStructMap[fSegmentationId].fMasterCloud.setWidth(aSamplePts.size());
    std::vector<cv::Vec3d> points;
    for (const auto& pt : aSamplePts) {
        points.emplace_back(pt[0], pt[1], fPathOnSliceIndex);
    }
    fSegStructMap[fSegmentationId].fMasterCloud.pushRow(points);

    fSegStructMap[fSegmentationId].fMinSegIndex = static_cast<int>(floor(fSegStructMap[fSegmentationId].fMasterCloud[0][2]));
    fSegStructMap[fSegmentationId].fMaxSegIndex = fSegStructMap[fSegmentationId].fMinSegIndex;
}

// Open volume package
void CWindow::OpenVolume()
{
    const QString defaultPathKey("default_path");

    QSettings settings;

    QString aVpkgPath = QString("");
    aVpkgPath = QFileDialog::getExistingDirectory(
        this, tr("Open Directory"), settings.value(defaultPathKey).toString(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    // Dialog box cancelled
    if (aVpkgPath.length() == 0) {
        vc::Logger()->info("Open .volpkg canceled");
        return;
    }

    // Checks the Folder Path for .volpkg extension
    auto const extension = aVpkgPath.toStdString().substr(
        aVpkgPath.toStdString().length() - 7, aVpkgPath.toStdString().length());
    if (extension != ".volpkg") {
        QMessageBox::warning(
            this, tr("ERROR"),
            "The selected file is not of the correct type: \".volpkg\"");
        vc::Logger()->error(
            "Selected file is not .volpkg: {}", aVpkgPath.toStdString());
        fVpkg = nullptr;  // Is need for User Experience, clears screen.
        return;
    }

    QDir currentDir;
    settings.setValue(defaultPathKey, currentDir.absoluteFilePath(aVpkgPath));

    // Open volume package
    if (!InitializeVolumePkg(aVpkgPath.toStdString() + "/")) {
        return;
    }

    // Check version number
    if (fVpkg->version() != VOLPKG_SUPPORTED_VERSION) {
        const auto msg = "Volume package is version " +
                         std::to_string(fVpkg->version()) +
                         " but this program requires version " +
                         std::to_string(VOLPKG_SUPPORTED_VERSION) + ".";
        vc::Logger()->error(msg);
        QMessageBox::warning(this, tr("ERROR"), QString(msg.c_str()));
        fVpkg = nullptr;
        return;
    }

    fVpkgPath = aVpkgPath;
    fPathOnSliceIndex = 0;
    currentVolume = fVpkg->volume();
    {
        const QSignalBlocker blocker{volSelect};
        volSelect->clear();
    }
    QStringList volIds;
    for (const auto& id : fVpkg->volumeIDs()) {
        volIds.append(QString::fromStdString(id));
    }
    volSelect->addItems(volIds);
}

void CWindow::CloseVolume(void)
{
    fVpkg = nullptr;
    fSegmentationId = "";
    fSegmentation = nullptr;
    currentVolume = nullptr;
    fWindowState = EWindowState::WindowStateIdle;  // Set Window State to Idle
    fPenTool->setChecked(false);                   // Reset PenTool Button
    fSegTool->setChecked(false);                   // Reset Segmentation Button
    ResetPointCloud();
    OpenSlice();
    InitPathList();
    UpdateView();
}

// Handle open request
void CWindow::Open(void)
{
    if (SaveDialog() == SaveResponse::Cancelled)
        return;

    CloseVolume();
    OpenVolume();
    OpenSlice();
    InitPathList();
    UpdateView();  // update the panel when volume package is loaded
}

// Close application
void CWindow::Close(void) { close(); }

// Pop up about dialog
void CWindow::Keybindings(void)
{
    // REVISIT - FILL ME HERE
    QMessageBox::information(
        this, tr("Keybindings for Volume Cartographer"),
        tr("A,D: Impact Range down/up by 1 \n"
        "[, ]: Alternative Impact Range down/up by 1 \n"
        "Arrow Left/Right: Slice down/up by 1 \n"
        "1,2: Slice down/up by 1 \n"
        "3,4: Slice down/up by 10 \n"
        "5,6: Slice down/up by 100 \n"
        "S: Segmentation Tool \n"
        "P: Pen Tool \n"
        "Space: Toggle Curve Visibility \n"
        "C: Alternate Toggle Curve Visibility \n"
        "Shift + Mouse Wheel: Zoom in/out \n"
        "Mouse Wheel: Scroll up/down \n"
        "Alt + Mouse Wheel: Scroll left/right \n"
        "Mouse Left Click: Draw Curve in Pen mode. Adjust Point in Segmentation mode \n"
        "Mouse Right Click: Snap Closest Point to Cursor \n"
        "Shift + Mouse Left Click: Alternative Snap Closest Point to Cursor \n"
        "Mouse Back/Forward Button: Follow Curve View"));
}

// Pop up about dialog
void CWindow::About(void)
{
    // REVISIT - FILL ME HERE
    QMessageBox::information(
        this, tr("About Volume Cartographer"),
        tr("Vis Center, University of Kentucky"));
}

// Save point cloud to path directory
void CWindow::SavePointCloud()
{
    int count = 0;
    int total = 0;
    for (auto& seg : fSegStructMap) {
        total++;
        auto& segStruct = seg.second;
        if (segStruct.fMasterCloud.empty() || segStruct.fSegmentationId.empty()) {
            qDebug() << "Empty cloud or segmentation ID to save for id " << segStruct.fSegmentationId.c_str();
            continue;
        }
        // Try to save cloud to volpkg
        try {
            segStruct.fSegmentation->setPointSet(segStruct.fMasterCloud);
            segStruct.fSegmentation->setVolumeID(currentVolume->id());
        } catch (std::exception& e) {
            QMessageBox::warning(
                this, "Error", "Failed to write cloud to volume package.");
            qDebug() << "Exception in save for id " << segStruct.fSegmentationId.c_str();
            continue;
        }
        count++;
    }

    std::string saveMessage = "Saved " + std::to_string(count) + " Volume Package(s) of " + std::to_string(total) + ".";
    const char* saveMessageChar = saveMessage.c_str();
    statusBar->showMessage(tr(saveMessageChar), 5000);
    vc::Logger()->info(saveMessageChar);
    fVpkgChanged = false;
}

// Create new path
void CWindow::OnNewPathClicked(void)
{
    // Save if we need to
    if (SaveDialog() == SaveResponse::Cancelled) {
        return;
    }

    // Make a new segmentation in the volpkg
    auto seg = fVpkg->newSegmentation();
    const auto newSegmentationId = seg->id();

    // Add a new path to the tree widget
    QTreeWidgetItem *newItem = new QTreeWidgetItem(fPathListWidget);
    newItem->setText(0, QString(newSegmentationId.c_str()));
    newItem->setCheckState(1, Qt::Unchecked);
    newItem->setCheckState(2, Qt::Unchecked);

    // Activate the new item
    fPathListWidget->setCurrentItem(newItem);
    ChangePathItem(newSegmentationId); // Creating new curve
    newItem->setCheckState(1, Qt::Checked); 
    newItem->setCheckState(2, Qt::Checked); 
    fSegStructMap[newSegmentationId].display = true;
    fSegStructMap[newSegmentationId].compute = true;
    UpdateView();
}

void CWindow::UpdateSegmentCheckboxes(std::string aSegID) {
    if (aSegID.empty()) {
        // qDebug() << "UpdateSegmentCheckboxes: aSegID is empty";
        return;
    }
    if (fSegStructMap[aSegID].display || fSegStructMap[aSegID].compute) {
        // Disable all other new and empty Segmentations if new Segmentation created
        if (!fSegStructMap[aSegID].fSegmentationId.empty() && fSegStructMap[aSegID].fMasterCloud.empty()) {
            // qDebug() << "Disable all other new and empty Segmentations";
            for(auto& seg : fSegStructMap) {
                if (!seg.second.fSegmentationId.empty() && seg.first != aSegID && seg.second.fMasterCloud.empty()) {
                    seg.second.display = false;
                    seg.second.compute = false;
                    // qDebug() << "Compute " << seg.first.c_str() << " set compute false. while clicked on " << aSegID.c_str();
                    // uncheck the checkbox
                    QList<QTreeWidgetItem*> previousItems = fPathListWidget->findItems(
                        QString(seg.first.c_str()), Qt::MatchExactly, 0);
                    if (!previousItems.isEmpty())
                    {
                        previousItems[0]->setCheckState(1, Qt::Unchecked);
                        previousItems[0]->setCheckState(2, Qt::Unchecked);
                    }
                }
            }
        }

        // Disable all empty Segmentations if Segmentation with point cloud is enabled
        if (!fSegStructMap[aSegID].fSegmentationId.empty() && !fSegStructMap[aSegID].fMasterCloud.empty()) {
            // qDebug() << "Disable all pen Segmentations";
            for(auto& seg : fSegStructMap) {
                if (!seg.second.fSegmentationId.empty() && seg.first != aSegID && seg.second.fMasterCloud.empty()) {
                    // qDebug() << "Disable " << seg.first.c_str() << " id " << seg.second.fSegmentationId.c_str() << " with current id segment clicked: " << aSegID.c_str();
                    seg.second.display = false;
                    // qDebug() << "Compute " << seg.first.c_str() << " set compute false. while clicked on " << aSegID.c_str();
                    seg.second.compute = false;
                    // uncheck the checkbox
                    QList<QTreeWidgetItem*> previousItems = fPathListWidget->findItems(
                        QString(seg.first.c_str()), Qt::MatchExactly, 0);
                    if (!previousItems.isEmpty())
                    {
                        previousItems[0]->setCheckState(1, Qt::Unchecked);
                        previousItems[0]->setCheckState(2, Qt::Unchecked);
                    }
                }
            }
        }
    }

    // Delete completely disabled Segmentations from fSegStructMap
    auto it = fSegStructMap.begin();
    while (it != fSegStructMap.end()) {
        if (!it->second.display && !it->second.compute) {
            it = fSegStructMap.erase(it);
        } else {
            ++it;
        }
    }
}

void CWindow::toggleDisplayAll(bool checked)
{
    std::string lastID;
    // Iterate through all the items in the QTreeWidget and update their state.
    QTreeWidgetItemIterator it(fPathListWidget);
    while (*it) {
        QTreeWidgetItem* item = *it;
        std::string aSegID = item->text(0).toStdString();
        lastID = aSegID;
        if (checked) {
            // If the button/checkbox for "Display All" is checked, set all items to "Checked" state.
            if (item->checkState(1) != Qt::Checked) {
                // Only call ChangePathItem if the state is actually changing.
                ChangePathItem(aSegID);
            }
            item->setCheckState(1, Qt::Checked);
            fSegStructMap[aSegID].display = true;
        } else {
            fchkComputeAll->setChecked(false); 
            // If the button/checkbox for "Display All" is unchecked, set all items to "Unchecked" state.
            item->setCheckState(1, Qt::Unchecked);
            item->setCheckState(2, Qt::Unchecked);
            std::string aSegID = item->text(0).toStdString();
            fSegStructMap[aSegID].display = false;
            fSegStructMap[aSegID].compute = false;
        }
        ++it;
    }
    UpdateSegmentCheckboxes(lastID);
    UpdateView(); // Assuming this function updates the display.
}

void CWindow::toggleComputeAll(bool checked)
{
    std::string lastID;
    // Iterate through all the items in the QTreeWidget and update their state.
    QTreeWidgetItemIterator it(fPathListWidget);
    while (*it) {
        QTreeWidgetItem* item = *it;
        std::string aSegID = item->text(0).toStdString();
        lastID = aSegID;
        if (checked) {
            if (item->checkState(1) != Qt::Checked) {
                // Only call ChangePathItem if the state is actually changing.
                ChangePathItem(aSegID);
            }
            fchkDisplayAll->setChecked(true);
            // If the button/checkbox for "Compute All" is checked, set all items to "Checked" state.
            item->setCheckState(1, Qt::Checked);
            item->setCheckState(2, Qt::Checked);
            fSegStructMap[aSegID].compute = true;
            // Also check "Display" because we can't compute without displaying.
            item->setCheckState(1, Qt::Checked);
            fSegStructMap[aSegID].display = true;
        } else {
            // If the button/checkbox for "Compute All" is unchecked, set all items to "Unchecked" state.
            item->setCheckState(2, Qt::Unchecked);
            std::string aSegID = item->text(0).toStdString();
            fSegStructMap[aSegID].compute = false;
        }
        ++it;
    }
    UpdateSegmentCheckboxes(lastID);
    UpdateView(); // Assuming this function updates the display.
}

// Handle path item click event
void CWindow::OnPathItemClicked(QTreeWidgetItem* item, int column)
{
    std::string aSegID = item->text(0).toStdString();
    // qDebug() << "Item clicked: " << item->text(0) << " Column: " << column;
    // If the first checkbox (in column 1) is clicked
    if (column == 0) // Highlight the curve
    {
        for(auto& seg : fSegStructMap) {
            seg.second.highlighted = false;
        }
        
        // Check if aSegID is in fSegStructMap
        if (fSegStructMap.find(aSegID) != fSegStructMap.end()) {
            fSegStructMap[aSegID].highlighted = true;
        }
    }
    else if (column == 1) // Display
    {
        if (item->checkState(column) == Qt::Checked)
        {
            if (SaveDialog() == SaveResponse::Cancelled)
            {
                // Update the list to show the previous selection
                QList<QTreeWidgetItem*> previousItems = fPathListWidget->findItems(
                    QString(fSegmentationId.c_str()), Qt::MatchExactly, 0);
                
                if (!previousItems.isEmpty())
                {
                    fPathListWidget->setCurrentItem(previousItems[0]);
                }
                
                // Uncheck the checkbox
                item->setCheckState(column, Qt::Unchecked);
            }
            // qDebug() << "Display " << aSegID.c_str();
            ChangePathItem(aSegID);
            // qDebug() << "Display " << aSegID.c_str() << " set display true.";
            fSegStructMap[aSegID].display = true;
        }
        else
        {
            // Also Uncheck the second checkbox (Compute). Never Compute without displaying the Curve.
            item->setCheckState(2, Qt::Unchecked);
            fSegStructMap[aSegID].display = false;
            // qDebug() << "Compute " << aSegID.c_str() << " set compute false.";
            fSegStructMap[aSegID].compute = false;
        }
    }
    // If the second checkbox (in column 2) is clicked
    else if (column == 2) // Compute
    {
        if (item->checkState(column) == Qt::Checked)
        {
            // Only compute if the first checkbox (Display) is checked, so check it too
            // Check the first checkbox
            if (item->checkState(1) != Qt::Checked)
            {
                item->setCheckState(1, Qt::Checked);
                ChangePathItem(aSegID);
            }
            fSegStructMap[aSegID].display = true;
            fSegStructMap[aSegID].compute = true;
            // qDebug() << "Compute " << aSegID.c_str() << " set compute true.";
        }
        else {
            // qDebug() << "Compute " << aSegID.c_str() << " set compute false.";
            fSegStructMap[aSegID].compute = false;
        }
    }

    UpdateSegmentCheckboxes(aSegID);

    UpdateView();
}

// Logic to activate pen tool
void CWindow::ActivatePenTool() {
    // Pen tool available
    if (fPenTool->isEnabled()) {
        fPenTool->setChecked(!fPenTool->isChecked());
        TogglePenTool();
    }
}

// Logic to activate/deactivate segmentation tool
void CWindow::ActivateSegmentationTool() {
    // Segmentation tool available
    if (fSegTool->isEnabled()) {
        fSegTool->setChecked(!fSegTool->isChecked());
        ToggleSegmentationTool();
    }
}

// Toggle the status of the pen tool
void CWindow::TogglePenTool(void)
{
    if (fPenTool->isChecked()) {
        fWindowState = EWindowState::WindowStateDrawPath;

        // turn off edit tool
        fSegTool->setChecked(false);
    } else {
        fWindowState = EWindowState::WindowStateIdle;

        if (fSplineCurve.GetNumOfControlPoints() > 1) {
            SetPathPointCloud();  // finished drawing, set up path
            SavePointCloud();
            SetUpCurves();
            OpenSlice();
            SetCurrentCurve(fPathOnSliceIndex);
        }
        fSplineCurve.Clear();
        fVolumeViewerWidget->ResetSplineCurve();
    }

    UpdateView();
}

// Toggle the status of the segmentation tool
void CWindow::ToggleSegmentationTool(void)
{
    if (fSegTool->isChecked()) {
        // If the prefetching worker is not yet running, start it
        if (!prefetchWorker.joinable()) {
            prefetchWorker = std::thread(&CWindow::prefetchSlices, this);
        }
        // Start prefetching around the current slice
        startPrefetching(fPathOnSliceIndex);

        fWindowState = EWindowState::WindowStateSegmentation;
        fUpperPart.reset();
        fStartingPath.clear();
        SplitCloud();

        // turn off edit tool
        fPenTool->setChecked(false);
    } else {
        CleanupSegmentation();
    }
    UpdateView();
}

void CWindow::OnChangeSegAlgo(int index)
{
    this->ui.segParamsStack->setCurrentIndex(index);
}

// Handle gravity value change
void CWindow::OnEdtAlphaValChange()
{
    bool aIsOk;
    double aNewVal = fEdtAlpha->text().toDouble(&aIsOk);
    if (aIsOk) {
        if (aNewVal <= 0.0) {
            aNewVal = 0.0;
            fEdtAlpha->setText(QString::number(aNewVal));
        } else if (aNewVal > 1.0) {
            aNewVal = 1.0;
            fEdtAlpha->setText(QString::number(aNewVal));
        }
        fSegParams.fAlpha = aNewVal;
    }
}

void CWindow::OnEdtBetaValChange()
{
    bool aIsOk;
    double aNewVal = fEdtBeta->text().toDouble(&aIsOk);
    if (aIsOk) {
        if (aNewVal <= 0.0) {
            aNewVal = 0.0;
            fEdtBeta->setText(QString::number(aNewVal));
        } else if (aNewVal > 1.0) {
            aNewVal = 1.0;
            fEdtBeta->setText(QString::number(aNewVal));
        }
        fSegParams.fBeta = aNewVal;
    }
}

void CWindow::OnEdtDeltaValChange()
{
    bool aIsOk;
    double aNewVal = fEdtDelta->text().toDouble(&aIsOk);
    if (aIsOk) {
        if (aNewVal <= 0.0) {
            aNewVal = 0.0;
            fEdtDelta->setText(QString::number(aNewVal));
        } else if (aNewVal > 1.0) {
            aNewVal = 1.0;
            fEdtDelta->setText(QString::number(aNewVal));
        }
        fSegParams.fDelta = aNewVal;
    }
}

void CWindow::OnEdtK1ValChange()
{
    bool aIsOk;
    double aNewVal = fEdtK1->text().toDouble(&aIsOk);
    if (aIsOk) {
        if (aNewVal <= 0.0) {
            aNewVal = 0.0;
            fEdtK1->setText(QString::number(aNewVal));
        } else if (aNewVal > 1.0) {
            aNewVal = 1.0;
            fEdtK1->setText(QString::number(aNewVal));
        }
        fSegParams.fK1 = aNewVal;
    }
}

void CWindow::OnEdtK2ValChange()
{
    bool aIsOk;
    double aNewVal = fEdtK2->text().toDouble(&aIsOk);
    if (aIsOk) {
        if (aNewVal <= 0.0) {
            aNewVal = 0.0;
            fEdtK2->setText(QString::number(aNewVal));
        } else if (aNewVal > 1.0) {
            aNewVal = 1.0;
            fEdtK2->setText(QString::number(aNewVal));
        }
        fSegParams.fK2 = aNewVal;
    }
}

void CWindow::OnEdtDistanceWeightChange()
{
    bool aIsOk;
    int aNewVal = fEdtDistanceWeight->text().toInt(&aIsOk);
    if (aIsOk) {
        if (aNewVal > 100) {
            aNewVal = 100;
        } else if (aNewVal < 0) {
            aNewVal = 0;
        }
        fEdtDistanceWeight->setText(QString::number(aNewVal));
        fSegParams.fPeakDistanceWeight = aNewVal;
    }
}

void CWindow::OnEdtWindowWidthChange(int newVal)
{
    fSegParams.fWindowWidth = newVal;
}

void CWindow::OnOptIncludeMiddleClicked(bool clicked)
{
    fOptIncludeMiddle->setChecked(clicked);
    fSegParams.fIncludeMiddle = clicked;
}

/*
// Handle sample distance value change
void CWindow::OnEdtSampleDistValChange( QString nText )
{
    // REVISIT - the widget should be disabled and the change ignored for now
    bool aIsOk;
    int aNewVal = nText.toInt( &aIsOk );
    if ( aIsOk ) {
        fSegParams.fThreshold = aNewVal;
    }
}
*/

// Handle starting slice value change
void CWindow::OnEdtStartingSliceValChange(QString /*nText*/)
{
    // REVISIT - FILL ME HERE
    // REVISIT - should be equivalent to "set current slice", the same as
    // navigation through slices
}

// Handle ending slice value change
void CWindow::OnEdtEndingSliceValChange()
{
    // ending slice index
    bool aIsOk = false;
    int aNewVal = fEdtEndIndex->displayText().toInt(&aIsOk);
    if (aIsOk &&
        aNewVal < currentVolume->numSlices()) {
        fEndTargetOffset = aNewVal - fPathOnSliceIndex;
    } else {
        statusBar->showMessage(
            tr("ERROR: Selected slice is out of range of the volume!"), 10000);
        fEdtEndIndex->setText(
            QString::number(fPathOnSliceIndex + fEndTargetOffset));
    }
}

// Handle start segmentation
void CWindow::OnBtnStartSegClicked(void) { DoSegmentation(); }

// Handle start segmentation
void CWindow::OnEdtImpactRange(int nImpactRange)
{
    fVolumeViewerWidget->SetImpactRange(nImpactRange);
    fLabImpactRange->setText(QString::number(nImpactRange));
}

// Handle loading any slice
void CWindow::OnLoadAnySlice(int nSliceIndex)
{
    if (nSliceIndex >= 0 && nSliceIndex < currentVolume->numSlices()) {
        fPathOnSliceIndex = nSliceIndex;
        OpenSlice();
        SetCurrentCurve(fPathOnSliceIndex);
        UpdateView();
    } else
        statusBar->showMessage(
            tr("ERROR: Selected slice is out of range of the volume!"), 10000);
}

// Handle loading the next slice
void CWindow::OnLoadNextSlice(void)
{
    int shift = (qga::keyboardModifiers() == Qt::ShiftModifier) ? 10 : 1;
    OnLoadNextSliceShift(shift);
}

void CWindow::OnLoadNextSliceShift(int shift)
{
    if (currentVolume == nullptr) {
        return;
    }
    if (fPathOnSliceIndex + shift >= currentVolume->numSlices()) {
        shift = currentVolume->numSlices() - fPathOnSliceIndex - 1;
    }

    if (shift != 0) {
        fPathOnSliceIndex += shift;
        OpenSlice();
        SetCurrentCurve(fPathOnSliceIndex);
        UpdateView();
    } else {
        statusBar->showMessage(tr("Already at the end of the volume!"), 10000);
    }
}

// Handle loading the previous slice
void CWindow::OnLoadPrevSlice(void)
{
    int shift = (qga::keyboardModifiers() == Qt::ShiftModifier) ? 10 : 1;
    OnLoadPrevSliceShift(shift);
}

void CWindow::OnLoadPrevSliceShift(int shift)
{
    if (fPathOnSliceIndex - shift < 0) {
        shift = fPathOnSliceIndex;
    }

    if (shift != 0) {
        fPathOnSliceIndex -= shift;
        OpenSlice();
        SetCurrentCurve(fPathOnSliceIndex);
        UpdateView();
    } else {
        statusBar->showMessage(
            tr("Already at the beginning of the volume!"), 10000);
    }
}

// Handle path change event
void CWindow::OnPathChanged(void)
{
    if (fWindowState == EWindowState::WindowStateSegmentation) {
        for (auto& seg : fSegStructMap) {
            // update current segStruct
            seg.second.OnPathChanged();
        }
    }
}

bool CWindow::can_change_volume_()
{
    // return fVpkg != nullptr && fVpkg->numberOfVolumes() > 1 &&
    //        (fSegStructMap[fSegmentationId].fSegmentation == nullptr || !fSegStructMap[fSegmentationId].fSegmentation->hasPointSet() ||
    //         !fSegStructMap[fSegmentationId].fSegmentation->hasVolumeID());

    bool canChange = fVpkg != nullptr && fVpkg->numberOfVolumes() > 1;
    for (auto& seg : fSegStructMap) {
            auto& segStruct = seg.second;
            canChange = canChange && (segStruct.fSegmentation == nullptr || !segStruct.fSegmentation->hasPointSet() || !segStruct.fSegmentation->hasVolumeID());
    }
    return canChange;
}
