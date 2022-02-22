#include "PPMProjectionViewerWidget.hpp"

PPMProjectionViewerWidget::PPMProjectionViewerWidget(
    const std::string& visualizePPMIntersection,
    const std::string& ppmImageOverlay)
    : ppmProjectionThread_(visualizePPMIntersection, ppmImageOverlay)
{
    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(
        &ppmProjectionThread_, &PPMProjectionThread::ranProjection, this,
        &ImageScrollArea::updatePixmap);
    this->setMinimumWidth(10);
    if (visualizePPMIntersection.empty()) {
        this->setVisible(false);
    }
}

void PPMProjectionViewerWidget::handleSliceChange(int sliceIdx)
{
    ppmProjectionThread_.runProjection(sliceIdx);
}
