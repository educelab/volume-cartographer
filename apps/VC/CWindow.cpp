// CWindow.cpp
// Chao Du 2014 Dec
#include "CWindow.h"
#include "CVolumeViewerWithCurve.h"
#include "UDataManipulateUtils.h"

#define _DEBUG

using namespace ChaoVis;

// Constructor
CWindow::CWindow(void)
    : fVpkg(nullptr)
    , fPathOnSliceIndex(0)
    , fVolumeViewerWidget(nullptr)
    , fPathListWidget(nullptr)
    , fPenTool(nullptr)
    , fSegTool(nullptr)
    , fWindowState(EWindowState::WindowStateIdle)
    , fSegmentationId("")
    , fMinSegIndex(VOLPKG_SLICE_MIN_INDEX)
    , fMaxSegIndex(VOLPKG_SLICE_MIN_INDEX)
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
    fSegParams.fEndOffset = 5;

    // create UI widgets
    CreateWidgets();

    // create menu
    CreateActions();
    CreateMenus();

    OpenSlice();
    UpdateView();

    update();
}

// Constructor with QRect windowSize
CWindow::CWindow(QRect windowSize)
    : fVpkg(nullptr)
    , fPathOnSliceIndex(0)
    , fVolumeViewerWidget(nullptr)
    , fPathListWidget(nullptr)
    , fPenTool(nullptr)
    , fSegTool(nullptr)
    , fWindowState(EWindowState::WindowStateIdle)
    , fSegmentationId("")
    , fMinSegIndex(VOLPKG_SLICE_MIN_INDEX)
    , fMaxSegIndex(VOLPKG_SLICE_MIN_INDEX)
{
    ui.setupUi(this);

    fVpkgChanged = false;

    int height = windowSize.height();
    int width = windowSize.width();

    // MIN DIMENSIONS
    window()->setMinimumHeight(height / 2);
    window()->setMinimumWidth(width / 2);
    // MAX DIMENSIONS
    window()->setMaximumHeight(height);
    window()->setMaximumWidth(width);

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
    fSegParams.fEndOffset = 5;

    // create UI widgets
    CreateWidgets();

    // create menu
    CreateActions();
    CreateMenus();

    OpenSlice();
    UpdateView();

    update();
}

// Destructor
CWindow::~CWindow(void) { deleteNULL(fVpkg); }

// Handle mouse press event
void CWindow::mousePressEvent(QMouseEvent *nEvent) {}

// Handle key press event
void CWindow::keyPressEvent(QKeyEvent *event)
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
    QWidget *aTabSegment = this->findChild<QWidget *>("tabSegment");
    assert(aTabSegment != nullptr);

    fVolumeViewerWidget = new CVolumeViewerWithCurve();

    QVBoxLayout *aWidgetLayout = new QVBoxLayout;
    aWidgetLayout->addWidget(fVolumeViewerWidget);

    aTabSegment->setLayout(aWidgetLayout);

    // pass the reference of the curve to the widget
    fVolumeViewerWidget->SetSplineCurve(fSplineCurve);
    fVolumeViewerWidget->SetIntersectionCurve(fIntersectionCurve);

    connect(
        fVolumeViewerWidget,
        SIGNAL(SendSignalOnNextClicked()),
        this,
        SLOT(OnLoadNextSlice()));
    connect(
        fVolumeViewerWidget,
        SIGNAL(SendSignalOnPrevClicked()),
        this,
        SLOT(OnLoadPrevSlice()));
    connect(
        fVolumeViewerWidget,
        SIGNAL(SendSignalOnLoadAnyImage(int)),
        this,
        SLOT(OnLoadAnySlice(int)));
    connect(
        fVolumeViewerWidget,
        SIGNAL(SendSignalPathChanged()),
        this,
        SLOT(OnPathChanged()));

    // new path button
    QPushButton *aBtnNewPath = this->findChild<QPushButton *>("btnNewPath");
    QPushButton *aBtnRemovePath =
        this->findChild<QPushButton *>("btnRemovePath");
    aBtnRemovePath->setEnabled(
        false);  // Currently no methods from removing paths
    connect(aBtnNewPath, SIGNAL(clicked()), this, SLOT(OnNewPathClicked()));

    // pen tool and edit tool
    fPenTool = this->findChild<QPushButton *>("btnPenTool");
    fSegTool = this->findChild<QPushButton *>("btnSegTool");
    connect(fPenTool, SIGNAL(clicked()), this, SLOT(TogglePenTool()));
    connect(fSegTool, SIGNAL(clicked()), this, SLOT(ToggleSegmentationTool()));

    // list of paths
    fPathListWidget = this->findChild<QListWidget *>("lstPaths");
    connect(
        fPathListWidget,
        SIGNAL(itemClicked(QListWidgetItem *)),
        this,
        SLOT(OnPathItemClicked(QListWidgetItem *)));

    // segmentation methods
    QComboBox *aSegMethodsComboBox =
        this->findChild<QComboBox *>("cmbSegMethods");
    aSegMethodsComboBox->addItem(tr("Local Reslice Particle Simulation"));

    fEdtAlpha = this->findChild<QLineEdit *>("edtAlphaVal");
    fEdtBeta = this->findChild<QLineEdit *>("edtBetaVal");
    fEdtDelta = this->findChild<QLineEdit *>("edtDeltaVal");
    fEdtK1 = this->findChild<QLineEdit *>("edtK1Val");
    fEdtK2 = this->findChild<QLineEdit *>("edtK2Val");
    fEdtDistanceWeight = this->findChild<QLineEdit *>("edtDistanceWeightVal");
    fEdtWindowWidth = this->findChild<QLineEdit *>("edtWindowWidthVal");
    fOptIncludeMiddle = this->findChild<QCheckBox *>("includeMiddleOpt");
    connect(
        fEdtAlpha,
        SIGNAL(editingFinished()),
        this,
        SLOT(OnEdtAlphaValChange()));
    connect(
        fEdtBeta, SIGNAL(editingFinished()), this, SLOT(OnEdtBetaValChange()));
    connect(
        fEdtDelta,
        SIGNAL(editingFinished()),
        this,
        SLOT(OnEdtDeltaValChange()));
    connect(fEdtK1, SIGNAL(editingFinished()), this, SLOT(OnEdtK1ValChange()));
    connect(fEdtK2, SIGNAL(editingFinished()), this, SLOT(OnEdtK2ValChange()));
    connect(
        fEdtDistanceWeight,
        SIGNAL(editingFinished()),
        this,
        SLOT(OnEdtDistanceWeightChange()));
    connect(
        fEdtWindowWidth,
        SIGNAL(editingFinished()),
        this,
        SLOT(OnEdtWindowWidthChange()));
    connect(
        fOptIncludeMiddle,
        SIGNAL(clicked(bool)),
        this,
        SLOT(OnOptIncludeMiddleClicked(bool)));

    fEdtStartIndex = this->findChild<QLineEdit *>("edtStartingSliceVal");
    fEdtEndIndex = this->findChild<QLineEdit *>("edtEndingSliceVal");
    connect(
        fEdtStartIndex,
        SIGNAL(textEdited(QString)),
        this,
        SLOT(OnEdtStartingSliceValChange(QString)));
    connect(
        fEdtEndIndex,
        SIGNAL(editingFinished()),
        this,
        SLOT(OnEdtEndingSliceValChange()));

    // start segmentation button
    QPushButton *aBtnStartSeg = this->findChild<QPushButton *>("btnStartSeg");
    connect(
        aBtnStartSeg, SIGNAL(clicked()), this, SLOT(OnBtnStartSegClicked()));

    // Impact Range slider
    QSlider *fEdtImpactRange = this->findChild<QSlider *>("sldImpactRange");
    connect(
        fEdtImpactRange,
        SIGNAL(valueChanged(int)),
        this,
        SLOT(OnEdtImpactRange(int)));
    fLabImpactRange = this->findChild<QLabel *>("labImpactRange");
    fLabImpactRange->setText(QString::number(fEdtImpactRange->value()));

    // Setup the status bar
    statusBar = this->findChild<QStatusBar *>("statusBar");
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
    fOpenVolAct = new QAction(tr("&Open volume..."), this);
    connect(fOpenVolAct, SIGNAL(triggered()), this, SLOT(Open()));
    fExitAct = new QAction(tr("E&xit..."), this);
    connect(fExitAct, SIGNAL(triggered()), this, SLOT(Close()));
    fAboutAct = new QAction(tr("&About..."), this);
    connect(fAboutAct, SIGNAL(triggered()), this, SLOT(About()));
    fSavePointCloudAct = new QAction(tr("&Save volume..."), this);
    connect(
        fSavePointCloudAct, SIGNAL(triggered()), this, SLOT(SavePointCloud()));
}

// Asks User to Save Data Prior to VC.app Exit
void CWindow::closeEvent(QCloseEvent *closing)
{
    if (SaveDialog() == SaveResponse::Continue) {
        closing->accept();
    } else {
        closing->ignore();
    }
}

void CWindow::setWidgetsEnabled(bool state)
{
    this->findChild<QGroupBox *>("grpVolManager")->setEnabled(state);
    this->findChild<QGroupBox *>("grpSeg")->setEnabled(state);
    this->findChild<QPushButton *>("btnSegTool")->setEnabled(state);
    this->findChild<QPushButton *>("btnPenTool")->setEnabled(state);
    this->findChild<QGroupBox *>("groupBox_4")->setEnabled(state);
    fVolumeViewerWidget->setButtonsEnabled(state);
}

bool CWindow::InitializeVolumePkg(const std::string &nVpkgPath)
{
    deleteNULL(fVpkg);

    try {
        fVpkg = new VolumePkg(nVpkgPath);
    } catch (...) {
        std::cerr << "VC::Error: Volume package failed to initialize."
                  << std::endl;
    }

    fVpkgChanged = false;

    if (fVpkg == nullptr) {
        std::cerr
            << "VC::Error: Cannot open volume package at specified location: "
            << nVpkgPath << std::endl;
        QMessageBox::warning(
            this,
            "Error",
            "Volume package failed to load. Package might be corrupt.");
        return false;
    }

    return true;
}

CWindow::SaveResponse CWindow::SaveDialog(void)
{
    // Return if nothing has changed
    if (!fVpkgChanged)
        return SaveResponse::Continue;

    QMessageBox::StandardButton response = QMessageBox::question(
        this,
        "Save changes?",
        tr("Changes will be lost! Save volume package before continuing?\n"),
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    switch (response) {
        case QMessageBox::Save:
            SavePointCloud();
            return SaveResponse::Continue;
        case QMessageBox::Discard:
            fVpkgChanged = false;
            return SaveResponse::Continue;
        case QMessageBox::Cancel:
            return SaveResponse::Cancelled;
        default:
            return SaveResponse::Cancelled;  // should never be reached
    }
}

// Update the widgets
void CWindow::UpdateView(void)
{
    if (fVpkg == nullptr) {
        setWidgetsEnabled(false);  // Disable Widgets for User
        this->findChild<QLabel *>("lblVpkgName")
            ->setText("No Volume Package Loaded");
        return;
    }

    setWidgetsEnabled(true);  // Enable Widgets for User

    // show volume package name
    this->findChild<QLabel *>("lblVpkgName")
        ->setText(QString(fVpkg->getPkgName().c_str()));

    // set widget accessibility properly based on the states: is drawing? is
    // editing?
    fEdtAlpha->setText(QString("%1").arg(fSegParams.fAlpha));
    fEdtBeta->setText(QString("%1").arg(fSegParams.fBeta));
    fEdtDelta->setText(QString("%1").arg(fSegParams.fDelta));
    fEdtK1->setText(QString("%1").arg(fSegParams.fK1));
    fEdtK2->setText(QString("%1").arg(fSegParams.fK2));
    fEdtDistanceWeight->setText(
        QString("%1").arg(fSegParams.fPeakDistanceWeight));
    fEdtWindowWidth->setText(QString("%1").arg(fSegParams.fWindowWidth));
    fEdtStartIndex->setText(QString("%1").arg(fPathOnSliceIndex));

    if (fSegParams.fEndOffset + fPathOnSliceIndex >= fVpkg->getNumberOfSlices())
        fSegParams.fEndOffset =
            (fVpkg->getNumberOfSlices() - 1) - fPathOnSliceIndex;
    fEdtEndIndex->setText(QString("%1").arg(
        fSegParams.fEndOffset + fPathOnSliceIndex));  // offset + starting index

    if (fIntersectionCurve.GetPointsNum() == 0) {  // no points in current slice
        fSegTool->setEnabled(false);
    } else {
        fSegTool->setEnabled(true);
    }

    if (fSegmentationId.length() != 0 &&    // segmentation selected
        fMasterCloud.empty()) {  // current cloud is empty
        fPenTool->setEnabled(true);
    } else {
        fPenTool->setEnabled(false);
    }

    // REVISIT - these two states should be mutually exclusive, we guarantee
    // this when we toggle the button, BUGGY!
    if (fWindowState == EWindowState::WindowStateIdle) {
        fVolumeViewerWidget->SetViewState(
            CVolumeViewerWithCurve::EViewState::ViewStateIdle);
        this->findChild<QGroupBox *>("grpVolManager")->setEnabled(true);
        this->findChild<QGroupBox *>("grpSeg")->setEnabled(false);
    } else if (fWindowState == EWindowState::WindowStateDrawPath) {
        fVolumeViewerWidget->SetViewState(
            CVolumeViewerWithCurve::EViewState::ViewStateDraw);
        this->findChild<QGroupBox *>("grpVolManager")->setEnabled(false);
        this->findChild<QGroupBox *>("grpSeg")->setEnabled(false);
    } else if (fWindowState == EWindowState::WindowStateSegmentation) {
        fVolumeViewerWidget->SetViewState(
            CVolumeViewerWithCurve::EViewState::ViewStateEdit);
        this->findChild<QGroupBox *>("grpVolManager")->setEnabled(false);
        this->findChild<QGroupBox *>("grpSeg")->setEnabled(
            true);  // segmentation can be done only when seg tool is selected
    } else {
        // something else
    }

    fEdtStartIndex->setEnabled(
        false);  // starting slice is always the current slice
    // fEdtSampleDist->setEnabled( false ); // currently we cannot let the user
    // change the sample distance

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
    fVpkg->setActiveSegmentation(fSegmentationId);

    // load proper point cloud
    fMasterCloud = fVpkg->openCloud();
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
    int aTotalNumOfImmutablePts =
        fMasterCloud.width() * (fPathOnSliceIndex - fMinSegIndex);
    std::vector<volcart::Point3d > points;
    int width_cnt = 0;
    for (int i = 0; i < aTotalNumOfImmutablePts; ++i) {
        if(width_cnt != fMasterCloud.width())
            points.push_back(fMasterCloud[i]);
        else
        {
            fUpperPart.pushRow(points);
            points.clear();
            points.push_back(fMasterCloud[i]);
            width_cnt = 0;
        }
        width_cnt++;
    }
    // resize so the parts can be concatenated

    // lower part, the starting slice
    std::vector<volcart::Point3d > points2;
    width_cnt = 0;
    for (int i = 0; i < fMasterCloud.width(); ++i) {
        if (fMasterCloud[i + aTotalNumOfImmutablePts][2] != -1) {
            if (width_cnt != fMasterCloud.width() ){
                points2.push_back(fMasterCloud[i + aTotalNumOfImmutablePts]);
            }

            else {
                fLowerPart.pushRow(points);
                points2.clear();
                points2.push_back(fMasterCloud[i + aTotalNumOfImmutablePts]);
                width_cnt = 0;
            }
            //Not sure if this goes with the loop or the if statemnt -HH
            width_cnt++;
        }

    }

    if (fLowerPart.width() != fMasterCloud.width()) {
        QMessageBox::information(
            this,
            tr("Error"),
            tr("Starting chain length has null points. "
               "Try segmenting from an earlier slice."));
        CleanupSegmentation();
        return;
    }
}

// Do segmentation given the starting point cloud
void CWindow::DoSegmentation(void)
{
    statusBar->clearMessage();

    // REVISIT - do we need to get the latest value from the widgets since we
    // constantly get the values?
    if (!SetUpSegParams()) {
        QMessageBox::information(
            this, tr("Info"), tr("Invalid parameter for segmentation"));
        return;
    }

    // 2) do segmentation from the starting slice
    volcart::segmentation::LocalResliceSegmentation segmenter(*fVpkg);
    fLowerPart = segmenter.segmentPath(
            volcart::OrderedPointSet<volcart::Point3d>(),
        fEdtStartIndex->text().toInt(),
        fEdtEndIndex->text().toInt() - 1,
        fSegParams.fNumIters,
        1,
        fSegParams.fAlpha,
        fSegParams.fK1,
        fSegParams.fK2,
        fSegParams.fBeta,
        fSegParams.fDelta,
        fSegParams.fPeakDistanceWeight,
        fSegParams.fIncludeMiddle,
        false,
        false);

    // 3) concatenate the two parts to form the complete point cloud

    fUpperPart.append(fLowerPart);
    fMasterCloud.append(fUpperPart);

    statusBar->showMessage(tr("Segmentation complete"));
    fVpkgChanged = true;
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

    aNewVal = fEdtWindowWidth->text().toInt(&aIsOk);
    if (aIsOk) {
        fSegParams.fWindowWidth = aNewVal;
    } else {
        return false;
    }

    fSegParams.fIncludeMiddle = fOptIncludeMiddle->isChecked();

    // ending slice index
    aNewVal = fEdtEndIndex->text().toInt(&aIsOk);
    if (aIsOk && aNewVal >= fPathOnSliceIndex &&
        aNewVal < fVpkg->getNumberOfSlices()) {
        fSegParams.fEndOffset =
            aNewVal - fPathOnSliceIndex;  // difference between the starting
                                          // slice and ending slice
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
        std::cerr << "VC::Warning: Point cloud for this segmentation is empty."
                  << std::endl;
        return;
    }
    fIntersections.clear();
    int minIndex, maxIndex;
    if (fMasterCloud.empty()) {
        minIndex = maxIndex = fPathOnSliceIndex;
    } else {
        volcart::Point3d min_p, max_p;
        min_p = fMasterCloud.min();
        max_p = fMasterCloud.max();
        minIndex = floor(fMasterCloud[0][2]);
        maxIndex = floor(max_p[2]);
    }
    void OnEdtAlphaValChange();

    fMinSegIndex = minIndex;
    fMaxSegIndex = maxIndex;

    // assign rows of particles to the curves
    for (int i = 0; i < fMasterCloud.height(); ++i) {
        CXCurve aCurve;
        for (int j = 0; j < fMasterCloud.width(); ++j) {
            int pointIndex = j + (i * fMasterCloud.width());
            aCurve.SetSliceIndex((int)floor(fMasterCloud[pointIndex][2]));
            aCurve.InsertPoint(Vec2<float>(
                fMasterCloud[pointIndex][0],
                fMasterCloud[pointIndex][1]));
        }
        fIntersections.push_back(aCurve);
    }
}

// Set the current curve
void CWindow::SetCurrentCurve(int nCurrentSliceIndex)
{
    int curveIndex = nCurrentSliceIndex - fMinSegIndex;
    if (curveIndex >= 0 && curveIndex < fIntersections.size() &&
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
        aImgMat = fVpkg->volume().getSliceDataCopy(fPathOnSliceIndex);
        aImgMat.convertTo(aImgMat, CV_8UC3, 1.0 / 256.0);
        cvtColor(aImgMat, aImgMat, CV_GRAY2BGR);
    } else
        aImgMat = cv::Mat::zeros(10, 10, CV_8UC3);

    QImage aImgQImage;
    aImgQImage = Mat2QImage(aImgMat);
    fVolumeViewerWidget->SetImage(aImgQImage);
    fVolumeViewerWidget->SetImageIndex(fPathOnSliceIndex);
}

// Initialize path list
void CWindow::InitPathList(void)
{
    fPathListWidget->clear();
    if (fVpkg != nullptr) {
        // show the existing paths
        for (size_t i = 0; i < fVpkg->getSegmentations().size(); ++i) {
            fPathListWidget->addItem(new QListWidgetItem(
                QString(fVpkg->getSegmentations()[i].c_str())));
        }
    }
}

// Update the Master cloud with the path we drew
void CWindow::SetPathPointCloud(void)
{
    // calculate the path and save that to aMasterCloud
    std::vector<cv::Vec2f> aSamplePts;
    fSplineCurve.GetSamplePoints(aSamplePts);

    volcart::Point3d point;
    volcart::OrderedPointSet<volcart::Point3d > aPathCloud;
    std::vector<volcart::Point3d > points;
    for (size_t i = 0; i < aSamplePts.size(); ++i) {
        point[0] = aSamplePts[i][0];
        point[1] = aSamplePts[i][1];
        point[2] = fPathOnSliceIndex;
        points.push_back(point);
    }
    aPathCloud.pushRow(points);

    fMasterCloud = aPathCloud;
    fMinSegIndex = floor(fMasterCloud[0][2]);
    fMaxSegIndex = fMinSegIndex;
}

// Open volume package
void CWindow::OpenVolume(void)
{
    QString aVpkgPath = QString("");
    aVpkgPath = QFileDialog::getExistingDirectory(
        this,
        tr("Open Directory"),
        QDir::homePath(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    // Dialog box cancelled
    if (aVpkgPath.length() == 0) {
        std::cerr << "VC::Message: Open volume package cancelled." << std::endl;
        return;
    }

    // Checks the Folder Path for .volpkg extension
    std::string extension = aVpkgPath.toStdString().substr(
        aVpkgPath.toStdString().length() - 7, aVpkgPath.toStdString().length());
    if (extension.compare(".volpkg") != 0) {
        QMessageBox::warning(
            this,
            tr("ERROR"),
            "The selected file is not of the correct type: \".volpkg\"");
        std::cerr << "VC::Error: Selected file: " << aVpkgPath.toStdString()
                  << " is of the wrong type." << std::endl;
        fVpkg = nullptr;  // Is need for User Experience, clears screen.
        return;
    }

    // Open volume package
    if (!InitializeVolumePkg(aVpkgPath.toStdString() + "/")) {
        return;
    }

    // Check version number
    if (fVpkg->getVersion() < 2) {
        std::cerr << "VC::Error: Volume package is version "
                  << fVpkg->getVersion()
                  << " but this program requires a version >= 2." << std::endl;
        QMessageBox::warning(
            this,
            tr("ERROR"),
            "Volume package is version " +
                QString::number(fVpkg->getVersion()) +
                " but this program requires a version >= 2.");
        fVpkg = nullptr;
        return;
    }

    fVpkgPath = aVpkgPath;
    fPathOnSliceIndex = 0;
    fSegParams.fWindowWidth =
        std::ceil(fVpkg->getMaterialThickness() / fVpkg->getVoxelSize());
}

void CWindow::CloseVolume(void)
{
    fVpkg = nullptr;
    fSegmentationId = "";
    fWindowState = EWindowState::WindowStateIdle;// Set Window State to Idle
    fPenTool->setChecked(false); // Reset PenTool Button
    fSegTool->setChecked(false); // Reset Segmentation Button
    ResetPointCloud();
    OpenSlice();
    InitPathList();
    UpdateView();
}

// Reset point cloud
void CWindow::ResetPointCloud(void)
{
    fMasterCloud.clear();
    fUpperPart.clear() ;
    fLowerPart.clear();
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
        this,
        tr("About Volume Cartographer"),
        tr("Vis Center, University of Kentucky"));
}

// Save point cloud to path directory
void CWindow::SavePointCloud(void)
{
    if (fMasterCloud.size() == 0) {
        std::cerr << "VC::message: Empty point cloud. Nothing to save."
                  << std::endl;
        return;
    }

    // Try to save cloud to volpkg
    if (fVpkg->saveCloud(fMasterCloud) != EXIT_SUCCESS) {
        QMessageBox::warning(
            this, "Error", "Failed to write cloud to volume package.");
        return;
    }

    // Only mesh if we have more than one iteration of segmentation
    if (fMasterCloud.height() <= 1) {
        std::cerr << "VC::message: Cloud height <= 1. Nothing to mesh."
                  << std::endl;
    } else {
        if (fVpkg->saveMesh(
                fMasterCloud) !=
            EXIT_SUCCESS) {
            QMessageBox::warning(
                this, "Error", "Failed to write mesh to volume package.");
            return;
        } else {
            std::cerr << "VC::message: Succesfully saved mesh." << std::endl;
        }
    }

    statusBar->showMessage(tr("Volume saved."), 5000);
    std::cerr << "VC::message: Volume saved." << std::endl;
    fVpkgChanged = false;
}

// Create new path
void CWindow::OnNewPathClicked(void)
{
    // Save if we need to
    if (SaveDialog() == SaveResponse::Cancelled)
        return;

    // Make a new segmentation in the volpkg
    std::string newSegmentationId = fVpkg->newSegmentation();

    // add new path to path list
    QListWidgetItem *aNewPath =
        new QListWidgetItem(QString(newSegmentationId.c_str()));
    fPathListWidget->addItem(aNewPath);

    // Make sure we stay on the current slice
    fMinSegIndex = fPathOnSliceIndex;

    // Activate the new item
    fPathListWidget->setCurrentItem(aNewPath);
    ChangePathItem(newSegmentationId);
}

// Handle path item click event
void CWindow::OnPathItemClicked(QListWidgetItem *nItem)
{
    if (SaveDialog() == SaveResponse::Cancelled) {
        // Update the list to show the previous selection
        QListWidgetItem *previous = fPathListWidget->findItems(
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
        fUpperPart.clear();
        fLowerPart.clear();
        SplitCloud();

        // turn off edit tool
        fPenTool->setChecked(false);
    } else {
        CleanupSegmentation();
    }
    UpdateView();
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

void CWindow::OnEdtWindowWidthChange()
{
    bool aIsOk;
    int aNewVal = fEdtWindowWidth->text().toInt(&aIsOk);
    if (aIsOk) {
        if (aNewVal > 20) {
            aNewVal = 20;
        } else if (aNewVal < 1) {
            aNewVal = 1;
        }
        fEdtWindowWidth->setText(QString::number(aNewVal));
        fSegParams.fWindowWidth = aNewVal;
    }
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
void CWindow::OnEdtStartingSliceValChange(QString nText)
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
        aNewVal < fVpkg->getNumberOfSlices()) {
        fSegParams.fEndOffset =
            aNewVal - fPathOnSliceIndex;  // difference between the starting
                                          // slice and ending slice
    } else {
        statusBar->showMessage(
            tr("ERROR: Selected slice is out of range of the volume!"), 10000);
        fEdtEndIndex->setText(
            QString::number(fPathOnSliceIndex + fSegParams.fEndOffset));
    }
}

// Handle start segmentation
void CWindow::OnBtnStartSegClicked(void)
{
    DoSegmentation();
    CleanupSegmentation();
    UpdateView();
}

// Handle start segmentation
void CWindow::OnEdtImpactRange(int nImpactRange)
{
    fVolumeViewerWidget->SetImpactRange(nImpactRange);
    fLabImpactRange->setText(QString::number(nImpactRange));
}

// Handle loading any slice
void CWindow::OnLoadAnySlice(int nSliceIndex)
{
    if (nSliceIndex >= 0 && nSliceIndex < fVpkg->getNumberOfSlices()) {
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
    if (fPathOnSliceIndex < fVpkg->getNumberOfSlices() - 1) {
        ++fPathOnSliceIndex;
        OpenSlice();
        SetCurrentCurve(fPathOnSliceIndex);
        UpdateView();
    } else
        statusBar->showMessage(tr("Already at the end of the volume!"), 10000);
}

// Handle loading the previous slice
void CWindow::OnLoadPrevSlice(void)
{
    if (fPathOnSliceIndex > 0) {
        --fPathOnSliceIndex;
        OpenSlice();
        SetCurrentCurve(fPathOnSliceIndex);
        UpdateView();
    } else
        statusBar->showMessage(
            tr("Already at the beginning of the volume!"), 10000);
}

// Handle path change event
void CWindow::OnPathChanged(void)
{
    std::vector<volcart::Point3d > points;
    if (fWindowState == EWindowState::WindowStateSegmentation) {
        // update current slice
        fLowerPart.clear();
        for (size_t i = 0; i < fIntersectionCurve.GetPointsNum(); ++i) {
            volcart::Point3d tempPt;
            tempPt[0] = fIntersectionCurve.GetPoint(i)[0];
            tempPt[1] = fIntersectionCurve.GetPoint(i)[1];
            tempPt[2] = fPathOnSliceIndex;
            points.push_back(tempPt);
        }
        fLowerPart.pushRow(points);
    }
}
