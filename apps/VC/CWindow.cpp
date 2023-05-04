// CWindow.cpp
// Chao Du 2014 Dec
#include "CWindow.hpp"

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
{
    ui.setupUi(this);

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
    worker_thread_.quit();
    worker_thread_.wait();
}

// Handle mouse press event
void CWindow::mousePressEvent(QMouseEvent* /*nEvent*/) {}

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

    fVolumeViewerWidget = new CVolumeViewerWithCurve();

    QVBoxLayout* aWidgetLayout = new QVBoxLayout;
    aWidgetLayout->addWidget(fVolumeViewerWidget);

    aTabSegment->setLayout(aWidgetLayout);

    // pass the reference of the curve to the widget
    fVolumeViewerWidget->SetSplineCurve(fSplineCurve);
    fVolumeViewerWidget->SetIntersectionCurve(fIntersectionCurve);

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
        if (fSegmentation == nullptr || fSegmentation->hasVolumeID()) {
            return;
        }
        fSegmentation->setVolumeID(currentVolume->id());
        UpdateView();
    });

    // pen tool and edit tool
    fPenTool = this->findChild<QPushButton*>("btnPenTool");
    fSegTool = this->findChild<QPushButton*>("btnSegTool");
    connect(fPenTool, SIGNAL(clicked()), this, SLOT(TogglePenTool()));
    connect(fSegTool, SIGNAL(clicked()), this, SLOT(ToggleSegmentationTool()));

    // list of paths
    fPathListWidget = this->findChild<QListWidget*>("lstPaths");
    connect(
        fPathListWidget, SIGNAL(itemClicked(QListWidgetItem*)), this,
        SLOT(OnPathItemClicked(QListWidgetItem*)));

    // segmentation methods
    auto* aSegMethodsComboBox = this->findChild<QComboBox*>("cmbSegMethods");
    aSegMethodsComboBox->addItem(tr("Local Reslice Particle Simulation"));
    connect(
        aSegMethodsComboBox, SIGNAL(currentIndexChanged(int)), this,
        SLOT(OnChangeSegAlgo(int)));

    // ADD NEW SEGMENTATION ALGORITHM NAMES HERE
    // aSegMethodsComboBox->addItem(tr("My custom algorithm"));

    // LRPS segmentation parameters
    // all of these are contained in the this->ui.lrpsParams
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

    // Setup the status bar
    statusBar = this->findChild<QStatusBar*>("statusBar");
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
    fHelpMenu->addAction(fAboutAct);

    menuBar()->addMenu(fFileMenu);
    menuBar()->addMenu(fHelpMenu);
}

// Create actions
void CWindow::CreateActions(void)
{
    fOpenVolAct = new QAction(tr("&Open volpkg..."), this);
    connect(fOpenVolAct, SIGNAL(triggered()), this, SLOT(Open()));
    fExitAct = new QAction(tr("E&xit..."), this);
    connect(fExitAct, SIGNAL(triggered()), this, SLOT(Close()));
    fAboutAct = new QAction(tr("&About..."), this);
    connect(fAboutAct, SIGNAL(triggered()), this, SLOT(About()));
    fSavePointCloudAct = new QAction(tr("&Save volpkg..."), this);
    connect(
        fSavePointCloudAct, SIGNAL(triggered()), this, SLOT(SavePointCloud()));
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
    } else {
        fEdtEndIndex->setText(
            QString::number(fPathOnSliceIndex + fEndTargetOffset));
    }

    fSegTool->setEnabled(fIntersectionCurve.GetPointsNum() > 0);
    fPenTool->setEnabled(!fSegmentationId.empty() && fMasterCloud.empty());

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

// Activate a specific segmentation by ID
void CWindow::ChangePathItem(std::string segID)
{
    statusBar->clearMessage();

    // Close the current segmentation
    ResetPointCloud();

    // Activate requested segmentation
    fSegmentationId = segID;
    fSegmentation = fVpkg->segmentation(fSegmentationId);

    // load proper point cloud
    if (fSegmentation->hasPointSet()) {
        fMasterCloud = fSegmentation->getPointSet();
    } else {
        fMasterCloud.reset();
    }

    if (fSegmentation->hasVolumeID()) {
        currentVolume = fVpkg->volume(fSegmentation->getVolumeID());
        volSelect->setCurrentText(
            QString::fromStdString(fSegmentation->getVolumeID()));
    }

    SetUpCurves();

    // Move us to the lowest slice index for the cloud
    fPathOnSliceIndex = fMinSegIndex;
    OpenSlice();
    SetCurrentCurve(fPathOnSliceIndex);

    UpdateView();
}

// Split fMasterCloud into fUpperCloud and fLowerCloud
void CWindow::SplitCloud(void)
{
    // Convert volume z-index to PointSet index
    auto pathIndex = fPathOnSliceIndex - fMinSegIndex;

    // Upper, "immutable" part
    if (fPathOnSliceIndex > fMinSegIndex) {
        fUpperPart = fMasterCloud.copyRows(0, pathIndex);
    } else {
        fUpperPart = vc::OrderedPointSet<cv::Vec3d>(fMasterCloud.width());
    }

    // Lower part, the starting path
    fStartingPath = fMasterCloud.getRow(pathIndex);

    // Remove silly -1 points if they exist
    fStartingPath.erase(
        std::remove_if(
            std::begin(fStartingPath), std::end(fStartingPath),
            [](auto e) { return e[2] == -1; }),
        std::end(fStartingPath));

    // Make sure the sizes match now
    if (fStartingPath.size() != fMasterCloud.width()) {
        QMessageBox::information(
            this, tr("Error"),
            tr("Starting chain length has null points. Try segmenting from an "
               "earlier slice."));
        CleanupSegmentation();
        return;
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
    // ADD OTHER SEGMENTER SETUP HERE. MATCH THE IDX TO THE IDX IN THE
    // DROPDOWN LIST

    // set common parameters
    segmenter->setChain(fStartingPath);
    segmenter->setVolume(currentVolume);

    // setup
    submitSegmentation(segmenter);
    setWidgetsEnabled(false);
    worker_progress_.show();
    worker_progress_updater_.start();
}

void CWindow::onSegmentationFinished(Segmenter::PointSet ps)
{
    setWidgetsEnabled(true);
    worker_progress_updater_.stop();
    worker_progress_.close();
    // 3) concatenate the two parts to form the complete point cloud
    fUpperPart.append(ps);
    fMasterCloud = fUpperPart;

    statusBar->showMessage(tr("Segmentation complete"));
    fVpkgChanged = true;

    CleanupSegmentation();
    UpdateView();
}

void CWindow::onSegmentationFailed(std::string s)
{
    vc::Logger()->error("Segmentation failed: {}", s);
    statusBar->showMessage(tr("Segmentation failed"));
    QMessageBox::critical(
        this, tr("VC"), QString::fromStdString("Segmentation failed:\n\n" + s));

    setWidgetsEnabled(true);
    worker_progress_updater_.stop();
    worker_progress_.close();
    CleanupSegmentation();
    UpdateView();
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
    if (aIsOk && aNewVal >= fPathOnSliceIndex &&
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
    if (fVpkg == nullptr || fMasterCloud.empty()) {
        statusBar->showMessage(tr("Selected point cloud is empty"));
        vc::Logger()->warn("Segmentation point cloud is empty");
        return;
    }
    fIntersections.clear();
    int minIndex, maxIndex;
    if (fMasterCloud.empty()) {
        minIndex = maxIndex = fPathOnSliceIndex;
    } else {
        minIndex = static_cast<int>(floor(fMasterCloud[0][2]));
        maxIndex = static_cast<int>(floor(fMasterCloud.max()[2]));
    }

    fMinSegIndex = minIndex;
    fMaxSegIndex = maxIndex;

    // assign rows of particles to the curves
    for (size_t i = 0; i < fMasterCloud.height(); ++i) {
        CXCurve aCurve;
        for (size_t j = 0; j < fMasterCloud.width(); ++j) {
            int pointIndex = j + (i * fMasterCloud.width());
            aCurve.SetSliceIndex(
                static_cast<int>(floor(fMasterCloud[pointIndex][2])));
            aCurve.InsertPoint(Vec2<double>(
                fMasterCloud[pointIndex][0], fMasterCloud[pointIndex][1]));
        }
        fIntersections.push_back(aCurve);
    }
}

// Set the current curve
void CWindow::SetCurrentCurve(int nCurrentSliceIndex)
{
    int curveIndex = nCurrentSliceIndex - fMinSegIndex;
    if (curveIndex >= 0 &&
        curveIndex < static_cast<int>(fIntersections.size()) &&
        fIntersections.size() != 0) {
        fIntersectionCurve = fIntersections[curveIndex];
    } else {
        CXCurve emptyCurve;
        fIntersectionCurve = emptyCurve;
    }
}

// Open slice
void CWindow::OpenSlice(void)
{
    cv::Mat aImgMat;
    if (fVpkg != nullptr) {
        aImgMat = currentVolume->getSliceDataCopy(fPathOnSliceIndex);
        aImgMat.convertTo(aImgMat, CV_8UC1, 1.0 / 256.0);
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
            fPathListWidget->addItem(new QListWidgetItem(QString(s.c_str())));
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
    fMasterCloud.setWidth(aSamplePts.size());
    std::vector<cv::Vec3d> points;
    for (const auto& pt : aSamplePts) {
        points.emplace_back(pt[0], pt[1], fPathOnSliceIndex);
    }
    fMasterCloud.pushRow(points);

    fMinSegIndex = static_cast<int>(floor(fMasterCloud[0][2]));
    fMaxSegIndex = fMinSegIndex;
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

// Reset point cloud
void CWindow::ResetPointCloud(void)
{
    fMasterCloud.reset();
    fUpperPart.reset();
    fStartingPath.clear();
    fIntersections.clear();
    CXCurve emptyCurve;
    fIntersectionCurve = emptyCurve;
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
    if (fMasterCloud.empty()) {
        vc::Logger()->debug("Empty point cloud. Nothing to save.");
        return;
    }

    // Try to save cloud to volpkg
    try {
        fSegmentation->setPointSet(fMasterCloud);
        fSegmentation->setVolumeID(currentVolume->id());
    } catch (std::exception& e) {
        QMessageBox::warning(
            this, "Error", "Failed to write cloud to volume package.");
        return;
    }

    statusBar->showMessage(tr("Volume Package saved."), 5000);
    vc::Logger()->info("Volume Package saved");
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

    // add new path to path list
    auto* aNewPath = new QListWidgetItem(QString(newSegmentationId.c_str()));
    fPathListWidget->addItem(aNewPath);

    // Make sure we stay on the current slice
    fMinSegIndex = fPathOnSliceIndex;

    // Activate the new item
    fPathListWidget->setCurrentItem(aNewPath);
    ChangePathItem(newSegmentationId);
}

// Handle path item click event
void CWindow::OnPathItemClicked(QListWidgetItem* nItem)
{
    if (SaveDialog() == SaveResponse::Cancelled) {
        // Update the list to show the previous selection
        QListWidgetItem* previous = fPathListWidget->findItems(
            QString(fSegmentationId.c_str()), Qt::MatchExactly)[0];
        fPathListWidget->setCurrentItem(previous);
        return;
    }

    ChangePathItem(nItem->text().toStdString());
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
    if (aIsOk && aNewVal > fPathOnSliceIndex &&
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
        // update current slice
        fStartingPath.clear();
        cv::Vec3d tempPt;
        for (size_t i = 0; i < fIntersectionCurve.GetPointsNum(); ++i) {
            tempPt[0] = fIntersectionCurve.GetPoint(i)[0];
            tempPt[1] = fIntersectionCurve.GetPoint(i)[1];
            tempPt[2] = fPathOnSliceIndex;
            fStartingPath.push_back(tempPt);
        }
    }
}

bool CWindow::can_change_volume_()
{
    return fVpkg != nullptr && fVpkg->numberOfVolumes() > 1 &&
           (fSegmentation == nullptr || !fSegmentation->hasPointSet() ||
            !fSegmentation->hasVolumeID());
}
