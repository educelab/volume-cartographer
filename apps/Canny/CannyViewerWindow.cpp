#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-magic-numbers"

#include <QApplication>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QMessageBox>
#include <opencv2/opencv.hpp>

#include "CannyViewerWindow.hpp"

CannyViewerWindow::CannyViewerWindow(
    volcart::CannySettings* settings,
    const volcart::Volume::Pointer& volume,
    QWidget* parent)
    : QMainWindow(parent)
    , settings_(settings)
    , numSlices_(volume->numSlices())
    , splitter_(new QSplitter)
    , blurSizeLabel_(new QLabel)
    , minThresholdLabel_(new QLabel)
    , maxThresholdLabel_(new QLabel)
    , apertureSizeLabel_(new QLabel)
    , sliceLabel_(new QLabel)
    , closingSizeLabel_(new QLabel)
    , projectionEdgeLabel_(new QLabel("Projection edge:"))
    , blurSizeSlider_(new QSlider(Qt::Horizontal))
    , minThresholdSlider_(new QSlider(Qt::Horizontal))
    , maxThresholdSlider_(new QSlider(Qt::Horizontal))
    , apertureSizeSlider_(new QSlider(Qt::Horizontal))
    , sliceSlider_(new QSlider(Qt::Horizontal))
    , contourCheckBox_(new QCheckBox("Contour"))
    , closingSizeSlider_(new QSlider(Qt::Horizontal))
    , bilateralCheckBox_(new QCheckBox("Bilateral"))
    , projectionEdgeComboBox_(new QComboBox)
    , midpointCheckBox_(new QCheckBox("Midpoint"))
    , normalizeCheckBox_(new QCheckBox("Normalize preview image"))
    , buttonLayout_(new QHBoxLayout)
    , okButton_(new QPushButton("OK"))
    , cancelButton_(new QPushButton("Cancel"))
    , sliderLayout_(new QVBoxLayout)
    , sliderWidget_(new QWidget)
    , sliceCannyViewerWidget_(new SliceCannyViewerWidget(volume))
{
    connect(
        sliceCannyViewerWidget_, &SliceCannyViewerWidget::sliceLoaded, this,
        &CannyViewerWindow::handleSettingsRequest);

    blurSizeSlider_->setMaximum(30);
    blurSizeSlider_->setValue(settings_->blurSize);
    connect(
        blurSizeSlider_, &QSlider::valueChanged, this,
        &CannyViewerWindow::handle_settings_change_);

    minThresholdSlider_->setMaximum(255);
    minThresholdSlider_->setValue(settings_->minThreshold);
    connect(
        minThresholdSlider_, &QSlider::valueChanged, this,
        &CannyViewerWindow::handle_settings_change_);

    maxThresholdSlider_->setMaximum(255);
    maxThresholdSlider_->setValue(settings_->maxThreshold);
    connect(
        maxThresholdSlider_, &QSlider::valueChanged, this,
        &CannyViewerWindow::handle_settings_change_);

    apertureSizeSlider_->setMaximum(2);
    apertureSizeSlider_->setValue(settings_->apertureSize);
    connect(
        apertureSizeSlider_, &QSlider::valueChanged, this,
        &CannyViewerWindow::handle_settings_change_);

    sliceSlider_->setMinimum(static_cast<int>(settings_->zMin));
    sliceSlider_->setMaximum(static_cast<int>(settings_->zMax));
    sliceSlider_->setValue(static_cast<int>(settings_->zMin));
    connect(
        sliceSlider_, &QSlider::valueChanged, this,
        &CannyViewerWindow::handle_slice_change_);

    contourCheckBox_->setChecked(settings_->contour);
    contourCheckBox_->setMinimumWidth(20);
    connect(
        contourCheckBox_, &QCheckBox::stateChanged, this,
        &CannyViewerWindow::handle_settings_change_);

    closingSizeSlider_->setMaximum(32);
    closingSizeSlider_->setValue(settings_->closingSize);
    connect(
        closingSizeSlider_, &QSlider::valueChanged, this,
        &CannyViewerWindow::handle_settings_change_);

    bilateralCheckBox_->setChecked(settings_->bilateral);
    bilateralCheckBox_->setMinimumWidth(20);
    connect(
        bilateralCheckBox_, &QCheckBox::stateChanged, this,
        &CannyViewerWindow::handle_settings_change_);

    projectionEdgeLabel_->setMinimumWidth(20);
    projectionEdgeComboBox_->addItem("Left");
    projectionEdgeComboBox_->addItem("Right");
    projectionEdgeComboBox_->addItem("Top");
    projectionEdgeComboBox_->addItem("Bottom");
    projectionEdgeComboBox_->addItem("Mesh normals");
    projectionEdgeComboBox_->addItem("Inverted mesh normals");
    projectionEdgeComboBox_->addItem("None");
    if (settings_->projectionFrom == 'L') {
        projectionEdgeComboBox_->setCurrentText("Left");
    } else if (settings_->projectionFrom == 'R') {
        projectionEdgeComboBox_->setCurrentText("Right");
    } else if (settings_->projectionFrom == 'T') {
        projectionEdgeComboBox_->setCurrentText("Top");
    } else if (settings_->projectionFrom == 'B') {
        projectionEdgeComboBox_->setCurrentText("Bottom");
    } else if (settings_->projectionFrom == 'M') {
        projectionEdgeComboBox_->setCurrentText("Mesh normals");
    } else if (settings_->projectionFrom == 'I') {
        projectionEdgeComboBox_->setCurrentText("Inverted mesh normals");
    } else if (settings_->projectionFrom == 'N') {
        projectionEdgeComboBox_->setCurrentText("None");
    }
    projectionEdgeComboBox_->setMinimumWidth(20);
    connect(
        projectionEdgeComboBox_, &QComboBox::currentTextChanged, this,
        &CannyViewerWindow::handle_settings_change_);

    midpointCheckBox_->setChecked(settings_->midpoint);
    midpointCheckBox_->setMinimumWidth(20);
    connect(
        midpointCheckBox_, &QCheckBox::stateChanged, this,
        &CannyViewerWindow::handle_settings_change_);

    normalizeCheckBox_->setChecked(false);
    normalizeCheckBox_->setMinimumWidth(20);
    connect(
        normalizeCheckBox_, &QCheckBox::checkStateChanged,
        sliceCannyViewerWidget_,
        &SliceCannyViewerWidget::handleNormalizeChange);

    okButton_->setMinimumWidth(10);
    cancelButton_->setMinimumWidth(10);
    // prevent the OK button from making the parent layout grow
    okButton_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    cancelButton_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    connect(okButton_, &QPushButton::clicked, this, &CannyViewerWindow::handle_ok_);
    connect(cancelButton_, &QPushButton::clicked, this, &CannyViewerWindow::handle_cancel_);

    // prevent buttons from growing the parent layout
    buttonLayout_->setContentsMargins(0, 0, 0, 0);
    buttonLayout_->setSpacing(0);

    buttonLayout_->addWidget(cancelButton_);
    buttonLayout_->addWidget(okButton_);

    sliderLayout_->addWidget(blurSizeLabel_);
    sliderLayout_->addWidget(blurSizeSlider_);
    sliderLayout_->addWidget(minThresholdLabel_);
    sliderLayout_->addWidget(minThresholdSlider_);
    sliderLayout_->addWidget(maxThresholdLabel_);
    sliderLayout_->addWidget(maxThresholdSlider_);
    sliderLayout_->addWidget(apertureSizeLabel_);
    sliderLayout_->addWidget(apertureSizeSlider_);
    sliderLayout_->addWidget(sliceLabel_);
    sliderLayout_->addWidget(sliceSlider_);
    sliderLayout_->addWidget(contourCheckBox_);
    sliderLayout_->addWidget(closingSizeLabel_);
    sliderLayout_->addWidget(closingSizeSlider_);
    sliderLayout_->addWidget(bilateralCheckBox_);
    sliderLayout_->addWidget(projectionEdgeLabel_);
    sliderLayout_->addWidget(projectionEdgeComboBox_);
    sliderLayout_->addWidget(midpointCheckBox_);
    sliderLayout_->addWidget(normalizeCheckBox_);
    sliderLayout_->addStretch();
    sliderLayout_->addLayout(buttonLayout_);
    sliderWidget_->setLayout(sliderLayout_);

    splitter_->addWidget(sliceCannyViewerWidget_);
    splitter_->addWidget(sliderWidget_);
    splitter_->setSizes({2, 1});

    setCentralWidget(splitter_);

    const float sizeRatio = 3.0 / 5.0;
    resize(QGuiApplication::primaryScreen()->availableSize() * sizeRatio);

    handle_slice_change_();
    handle_settings_change_();
}

void CannyViewerWindow::handleSettingsRequest()
{
    sliceCannyViewerWidget_->handleSettingsChange(*settings_);
}

void CannyViewerWindow::handle_slice_change_()
{
    sliceLabel_->setText("Slice: " + QString::number(sliceSlider_->value()));
    sliceCannyViewerWidget_->handleSliceChange(sliceSlider_->value());
}

void CannyViewerWindow::handle_settings_change_()
{
    settings_->blurSize = blurSizeSlider_->value();
    settings_->minThreshold = minThresholdSlider_->value();
    settings_->maxThreshold = maxThresholdSlider_->value();
    settings_->closingSize = closingSizeSlider_->value();
    settings_->apertureSize = apertureSizeSlider_->value();
    settings_->contour = contourCheckBox_->isChecked();
    settings_->bilateral = bilateralCheckBox_->isChecked();
    if (projectionEdgeComboBox_->currentText() == "Left") {
        settings_->projectionFrom = 'L';
    } else if (projectionEdgeComboBox_->currentText() == "Right") {
        settings_->projectionFrom = 'R';
    } else if (projectionEdgeComboBox_->currentText() == "Top") {
        settings_->projectionFrom = 'T';
    } else if (projectionEdgeComboBox_->currentText() == "Bottom") {
        settings_->projectionFrom = 'B';
    } else if (projectionEdgeComboBox_->currentText() == "Mesh normals") {
        settings_->projectionFrom = 'M';
    } else if (projectionEdgeComboBox_->currentText() == "Inverted mesh normals") {
        settings_->projectionFrom = 'I';
    } else if (projectionEdgeComboBox_->currentText() == "None") {
        settings_->projectionFrom = 'N';
    }
    settings_->midpoint = midpointCheckBox_->isChecked();

    blurSizeLabel_->setText(
        "Blur size: " + QString::number(settings_->blurSize));
    minThresholdLabel_->setText(
        "Min threshold: " + QString::number(settings_->minThreshold));
    maxThresholdLabel_->setText(
        "Max threshold: " + QString::number(settings_->maxThreshold));
    closingSizeLabel_->setText(
        "Closing size: " + QString::number(settings_->closingSize));
    apertureSizeLabel_->setText(
        "Aperture size: " + QString::number(settings_->apertureSize));

    closingSizeLabel_->setVisible(settings_->contour);
    closingSizeSlider_->setVisible(settings_->contour);

    sliceCannyViewerWidget_->handleSettingsChange(*settings_);
}

void CannyViewerWindow::handle_ok_() { QApplication::exit(EXIT_SUCCESS); }

void CannyViewerWindow::handle_cancel_() { QApplication::exit(EXIT_FAILURE); }

void CannyViewerWindow::closeEvent(QCloseEvent*  /*event*/)
{
    const auto reply = QMessageBox::question(
        this, "Close", "Perform Canny edge segmentation?",
        QMessageBox::No | QMessageBox::Yes, QMessageBox::Yes);
    if (reply == QMessageBox::Yes) {
        QApplication::exit(EXIT_SUCCESS);
    } else {
        QApplication::exit(EXIT_FAILURE);
    }
}

#pragma clang diagnostic pop
