#include <QGuiApplication>
#include <QScrollBar>
#include <QWheelEvent>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "SliceCannyViewerWidget.hpp"

#include <vc/core/util/Logging.hpp>

namespace
{
auto Percentile(const cv::Mat& a, const float perc) -> float
{
    int histSize{256};
    float range[] = {0, 256};
    const float* histRange[] = {range};
    const int numPixs = a.rows * a.cols;
    const float pixReq = static_cast<float>(numPixs) * perc;
    cv::Mat hist;

    cv::calcHist(
        &a, 1, nullptr, cv::Mat(), hist, 1, &histSize, histRange, true, false);

    float cnt{0};
    for (int i = 0; i < histSize; i++) {
        cnt += hist.at<float>(i);
        if (cnt >= pixReq) {
            return static_cast<float>(i);
        }
    }
    return 255.f;
}

auto Window(const cv::Mat& a, const float low, const float high)
{
    cv::Mat b;
    a.convertTo(
        b, a.depth(), 255.f / (high - low), -low * 255.f / (high - low));
    return b;
}
}  // namespace

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
        &SliceCannyViewerWidget::update_canny_image_);
}

void SliceCannyViewerWidget::update_slice_image_(const cv::Mat& mat)
{
    originalSliceMat_ = mat;
    displayMat_ = originalSliceMat_.clone();

    // Normalize the src
    if (normalize_) {
        const auto low = Percentile(originalSliceMat_, 0.01f);
        const auto high = Percentile(originalSliceMat_, 0.99f);
        displayMat_ = Window(originalSliceMat_, low, high);
    }

    cv::cvtColor(displayMat_, displayMat_, cv::COLOR_GRAY2BGR);

    volcart::Logger()->info("update_slice_image");
    ImageScrollArea::updatePixmap(displayMat_);
    emit sliceLoaded();
}

void SliceCannyViewerWidget::update_canny_image_(const cv::Mat& mat)
{
    volcart::Logger()->info("update_canny_image");
    updatePixmap(mat);
}

void SliceCannyViewerWidget::handleSettingsChange(
    volcart::CannySettings cannySettings)
{
    volcart::Logger()->info("handleSettingsChange");
    cannyThread_.runCanny(
        originalSliceMat_, displayMat_, std::move(cannySettings));
}

void SliceCannyViewerWidget::handleSliceChange(int sliceIdx)
{
    fetchSliceThread_.fetchSlice(sliceIdx);
}

void SliceCannyViewerWidget::handleNormalizeChange(const Qt::CheckState state)
{
    // TODO: The display images flashes bright briefly when changing settings
    normalize_ = state == Qt::CheckState::Checked;
    volcart::Logger()->info("handleNormalizeChange");
    update_slice_image_(originalSliceMat_);
}
