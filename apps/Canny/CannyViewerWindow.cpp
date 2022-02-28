#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-magic-numbers"

#include <QGuiApplication>
#include <QScreen>
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
    , blurSizeSlider_(new QSlider(Qt::Horizontal))
    , minThresholdSlider_(new QSlider(Qt::Horizontal))
    , maxThresholdSlider_(new QSlider(Qt::Horizontal))
    , apertureSizeSlider_(new QSlider(Qt::Horizontal))
    , sliceSlider_(new QSlider(Qt::Horizontal))
    , contourCheckBox_(new QCheckBox("Contour"))
    , closingSizeSlider_(new QSlider(Qt::Horizontal))
    , bilateralCheckBox_(new QCheckBox("Bilateral"))
    , sliderLayout_(new QVBoxLayout)
    , sliderWidget_(new QWidget)
    , sliceCannyViewerWidget_(new SliceCannyViewerWidget(volume))
{
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

    sliceSlider_->setMaximum(numSlices_ - 1);
    sliceSlider_->setValue(0);
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
    sliderLayout_->addStretch();
    sliderWidget_->setLayout(sliderLayout_);

    splitter_->addWidget(sliceCannyViewerWidget_);
    splitter_->addWidget(sliderWidget_);
    splitter_->setSizes({2, 1});

    setCentralWidget(splitter_);

    const float sizeRatio = 3.0 / 5.0;
    resize(QGuiApplication::primaryScreen()->availableSize() * sizeRatio);

    // TODO not have to call these by having a settings widget that does it on
    // its own like in Projection app
    handle_slice_change_();
    handle_settings_change_();
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

#pragma clang diagnostic pop
