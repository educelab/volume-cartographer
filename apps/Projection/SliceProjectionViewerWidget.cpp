#include <QGuiApplication>
#include <QScrollBar>
#include <QWheelEvent>
#include <utility>

#include "SliceProjectionViewerWidget.hpp"

SliceProjectionViewerWidget::SliceProjectionViewerWidget(
    volcart::Volume::Pointer& volume,
    vtkSmartPointer<vtkPlane> cutPlane,
    vtkSmartPointer<vtkStripper> stripper)
    : fetchSliceThread_(volume)
    , projectionThread_(std::move(cutPlane), std::move(stripper))
{
    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(
        &fetchSliceThread_, &volcart::gui::FetchSliceThread::fetchedSlice, this,
        &SliceProjectionViewerWidget::update_slice_image_);
    connect(
        &projectionThread_, &SliceProjectionThread::ranProjection, this,
        &ImageScrollArea::updatePixmap);
}

void SliceProjectionViewerWidget::update_slice_image_(const cv::Mat& mat)
{
    originalSliceMat_ = mat;
    cv::cvtColor(originalSliceMat_, originalSliceMat_, cv::COLOR_GRAY2BGR);
    ImageScrollArea::updatePixmap(originalSliceMat_);
    emit sliceLoaded();
}

void SliceProjectionViewerWidget::handleSettingsChange(
    volcart::ProjectionSettings projectionSettings, int sliceIdx)
{
    projectionThread_.runProjection(
        originalSliceMat_, std::move(projectionSettings), sliceIdx);
}

void SliceProjectionViewerWidget::handleSliceChange(int sliceIdx)
{
    fetchSliceThread_.fetchSlice(sliceIdx);
}
