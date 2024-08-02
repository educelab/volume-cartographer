#include <QGuiApplication>
#include <QScrollBar>
#include <QWheelEvent>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "SliceCannyViewerWidget.hpp"

#include <vc/core/util/Logging.hpp>

SliceCannyViewerWidget::SliceCannyViewerWidget(
    const volcart::Volume::Pointer& volume)
    : fetchSliceThread_(volume)
{
    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(
        &fetchSliceThread_, &volcart::gui::FetchSliceThread::fetchedSlice, this,
        &SliceCannyViewerWidget::update_slice_image_);
    connect(
        &cannyThread_, &CannyThread::ranCanny, this,
        &ImageScrollArea::updatePixmap);
}

void SliceCannyViewerWidget::update_slice_image_(const cv::Mat& mat)
{
    originalSliceMat_ = mat;
    cv::cvtColor(originalSliceMat_, originalSliceMat_, cv::COLOR_GRAY2BGR);
    // It feels like we should update the viewer here, but: this.sliceLoaded ->
    // CannyViewerWindow::handleSettingsRequest -> this.handleSettingsChange ->
    // this.updatePixmap, which avoids flickering.
    emit sliceLoaded();
}

void SliceCannyViewerWidget::handleSettingsChange(CannySettings cannySettings)
{
    cannyThread_.runCanny(originalSliceMat_, std::move(cannySettings));
}

void SliceCannyViewerWidget::handleSliceChange(int sliceIdx)
{
    fetchSliceThread_.fetchSlice(sliceIdx);
}
