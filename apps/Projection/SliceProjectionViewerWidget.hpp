#pragma once

#include <QLabel>
#include <QScrollArea>
#include <QWidget>
#include <opencv2/core.hpp>
#include <vtkPlane.h>
#include <vtkStripper.h>

#include "SliceProjectionThread.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/gui_support/FetchSliceThread.hpp"
#include "vc/gui_support/ImageScrollArea.hpp"

class SliceProjectionViewerWidget : public volcart::gui::ImageScrollArea
{
    // clang-format off
    Q_OBJECT
    // clang-format on

public:
    SliceProjectionViewerWidget(
        volcart::Volume::Pointer& volume,
        vtkSmartPointer<vtkPlane> cutPlane,
        vtkSmartPointer<vtkStripper> stripper);

signals:
    void sliceLoaded();

public slots:
    void handleSliceChange(int sliceIdx);
    void handleSettingsChange(
        volcart::ProjectionSettings projectionSettings, int sliceIdx);

private slots:
    void update_slice_image_(const cv::Mat& mat);

private:
    volcart::gui::FetchSliceThread fetchSliceThread_;
    SliceProjectionThread projectionThread_;

    cv::Mat originalSliceMat_;
};
