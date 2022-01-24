#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-magic-numbers"

#include <QGuiApplication>
#include <QScreen>
#include <utility>
#include <opencv2/opencv.hpp>

#include "CannyViewer.hpp"

FetchSliceThread::FetchSliceThread(
    volcart::Volume::Pointer volume, QObject* parent)
    : QThread(parent), volume_(std::move(volume))
{
}

FetchSliceThread::~FetchSliceThread()
{
    mutex_.lock();
    abort_ = true;
    condition_.wakeOne();
    mutex_.unlock();
    wait();
}

void FetchSliceThread::fetchSlice(int sliceIdx)
{
    QMutexLocker locker(&mutex_);

    sliceIdx_ = sliceIdx;

    if (!isRunning()) {
        start(LowPriority);
    } else {
        restart_ = true;
        condition_.wakeOne();
    }
}

void FetchSliceThread::run()
{
    forever
    {
        if (abort_) {
            return;
        }

        mutex_.lock();
        const int sliceIdx = sliceIdx_;
        mutex_.unlock();

        if (!restart_) {
            const auto slice = volume_->getSliceDataCopy(sliceIdx);
            auto src = volcart::QuantizeImage(slice, CV_8U, false);
            emit fetchedSlice(src);
        }

        mutex_.lock();
        if (!restart_) {
            condition_.wait(&mutex_);
        }
        restart_ = false;
        mutex_.unlock();
    }
}

CannyThread::CannyThread(QObject* parent) : QThread(parent) {}

CannyThread::~CannyThread()
{
    mutex_.lock();
    abort_ = true;
    condition_.wakeOne();
    mutex_.unlock();
    wait();
}

void CannyThread::runCanny(cv::Mat& mat, volcart::CannySettings settings)
{
    QMutexLocker locker(&mutex_);

    mat_ = mat;
    settings_ = settings;

    if (!isRunning()) {
        start(LowPriority);
    } else {
        restart_ = true;
        condition_.wakeOne();
    }
}

void CannyThread::run()
{
    forever
    {
        if (abort_) {
            return;
        }

        mutex_.lock();
        cv::Mat src = mat_;
        volcart::CannySettings settings = settings_;
        mutex_.unlock();

        if (!restart_) {
            // Get the canny edges
            cv::Mat dst = volcart::Canny(src, settings);
            // Draw them on the slice
            src = volcart::ColorConvertImage(src, dst.channels());
            cv::addWeighted(src, 0.5, dst, 0.5, 0, dst);
            // Emit that pixmap to be rendered
            emit ranCanny(dst);
        }

        mutex_.lock();
        if (!restart_) {
            condition_.wait(&mutex_);
        }
        restart_ = false;
        mutex_.unlock();
    }
}

CannyViewer::CannyViewer(
    volcart::CannySettings* settings,
    const volcart::Volume::Pointer& volume,
    QWidget* parent)
    : QMainWindow(parent)
    , settings_(settings)
    , numSlices_(volume->numSlices())
    , scaleFactor_(1.0)
    , imageLabel_(new QLabel)
    , scrollArea_(new QScrollArea)
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
    , fetchSliceThread_(volume)
{
    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(
        &fetchSliceThread_, &FetchSliceThread::fetchedSlice, this,
        &CannyViewer::update_slice_image_);
    connect(
        &cannyThread_, &CannyThread::ranCanny, this,
        &CannyViewer::update_pixmap_);

    imageLabel_->setBackgroundRole(QPalette::Base);
    imageLabel_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    imageLabel_->setScaledContents(true);

    scrollArea_->setBackgroundRole(QPalette::Dark);
    scrollArea_->setWidget(imageLabel_);
    scrollArea_->verticalScrollBar()->installEventFilter(this);

    blurSizeSlider_->setMaximum(30);
    blurSizeSlider_->setValue(settings_->blurSize);
    connect(
        blurSizeSlider_, &QSlider::valueChanged, this,
        &CannyViewer::handle_settings_change_);

    minThresholdSlider_->setMaximum(255);
    minThresholdSlider_->setValue(settings_->minThreshold);
    connect(
        minThresholdSlider_, &QSlider::valueChanged, this,
        &CannyViewer::handle_settings_change_);

    maxThresholdSlider_->setMaximum(255);
    maxThresholdSlider_->setValue(settings_->maxThreshold);
    connect(
        maxThresholdSlider_, &QSlider::valueChanged, this,
        &CannyViewer::handle_settings_change_);

    apertureSizeSlider_->setMaximum(2);
    apertureSizeSlider_->setValue(settings_->apertureSize);
    connect(
        apertureSizeSlider_, &QSlider::valueChanged, this,
        &CannyViewer::handle_settings_change_);

    sliceSlider_->setMaximum(numSlices_ - 1);
    sliceSlider_->setValue(0);
    connect(
        sliceSlider_, &QSlider::valueChanged, this,
        &CannyViewer::handle_slice_change_);

    contourCheckBox_->setChecked(settings_->contour);
    contourCheckBox_->setMinimumWidth(20);
    connect(
        contourCheckBox_, &QCheckBox::stateChanged, this,
        &CannyViewer::handle_settings_change_);

    closingSizeSlider_->setMaximum(32);
    closingSizeSlider_->setValue(settings_->closingSize);
    connect(
        closingSizeSlider_, &QSlider::valueChanged, this,
        &CannyViewer::handle_settings_change_);

    bilateralCheckBox_->setChecked(settings_->bilateral);
    bilateralCheckBox_->setMinimumWidth(20);
    connect(
        bilateralCheckBox_, &QCheckBox::stateChanged, this,
        &CannyViewer::handle_settings_change_);

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

    splitter_->addWidget(scrollArea_);
    splitter_->addWidget(sliderWidget_);
    splitter_->setSizes({2, 1});

    setCentralWidget(splitter_);

    const float sizeRatio = 3.0 / 5.0;
    resize(QGuiApplication::primaryScreen()->availableSize() * sizeRatio);

    handle_slice_change_();
}

void CannyViewer::update_slice_image_(const cv::Mat& mat)
{
    originalSliceMat_ = mat;
    bool isFirstImage = false;
    if (imageLabel_->pixmap(Qt::ReturnByValue).isNull()) {
        isFirstImage = true;
    }
    update_pixmap_(mat);
    if (isFirstImage) {
        auto windowWidth = static_cast<float>(this->width());
        auto estimatedScrollAreaWidth = windowWidth * 2 / 3;
        scaleFactor_ =
            estimatedScrollAreaWidth /
            static_cast<float>(imageLabel_->pixmap(Qt::ReturnByValue).width());
        imageLabel_->resize(
            scaleFactor_ * imageLabel_->pixmap(Qt::ReturnByValue).size());
    }
    handle_settings_change_();
}

void CannyViewer::update_pixmap_(const cv::Mat& mat)
{
    QImage image = QImage(
        mat.data, mat.cols, mat.rows, static_cast<int>(mat.step),
        QImage::Format_Grayscale8);
    QPixmap pixmap = QPixmap::fromImage(image);
    imageLabel_->setPixmap(pixmap);
}

void CannyViewer::handle_slice_change_()
{
    sliceLabel_->setText("Slice: " + QString::number(sliceSlider_->value()));
    fetchSliceThread_.fetchSlice(sliceSlider_->value());
}

void CannyViewer::handle_settings_change_()
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

    cannyThread_.runCanny(originalSliceMat_, *settings_);
}

// https://stackoverflow.com/questions/22053102/scrollbar-captures-wheel-event-by-focus
auto CannyViewer::eventFilter(QObject* obj, QEvent* event) -> bool
{
    if (event->type() == QEvent::Wheel) {
        auto* wEvent = dynamic_cast<QWheelEvent*>(event);

        if (obj == scrollArea_->verticalScrollBar()) {
            wheelEvent(wEvent);
            return true;
        }
    }

    return false;
}

void CannyViewer::wheelEvent(QWheelEvent* event)
{
    int numDegrees = event->angleDelta().y() / 8;

    if (numDegrees > 0) {
        zoom_in_();
    } else if (numDegrees < 0) {
        zoom_out_();
    }

    event->accept();
}

void CannyViewer::zoom_in_() { scale_image_(1.25); }

void CannyViewer::zoom_out_() { scale_image_(0.8); }

void CannyViewer::scale_image_(double factor)
{
    scaleFactor_ *= factor;
    imageLabel_->resize(
        scaleFactor_ * imageLabel_->pixmap(Qt::ReturnByValue).size());

    AdjustScrollBar(scrollArea_->horizontalScrollBar(), factor);
    AdjustScrollBar(scrollArea_->verticalScrollBar(), factor);
}

void CannyViewer::AdjustScrollBar(QScrollBar* scrollBar, double factor)
{
    scrollBar->setValue(
        int(factor * scrollBar->value() +
            ((factor - 1) * scrollBar->pageStep() / 2)));
}
#pragma clang diagnostic pop
