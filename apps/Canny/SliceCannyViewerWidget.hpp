#pragma once

#include <QLabel>
#include <QScrollArea>
#include <QWidget>
#include <opencv2/core.hpp>
#include <vtkPlane.h>
#include <vtkStripper.h>

#include "CannyThread.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/gui_support/FetchSliceThread.hpp"
#include "vc/gui_support/ImageScrollArea.hpp"

class SliceCannyViewerWidget : public volcart::gui::ImageScrollArea
{
    // clang-format off
    Q_OBJECT
    // clang-format on

public:
    SliceCannyViewerWidget(const volcart::Volume::Pointer& volume);

signals:
    void sliceLoaded();

public slots:
    void handleSliceChange(int sliceIdx);
    void handleSettingsChange(volcart::CannySettings cannySettings);

private slots:
    void update_slice_image_(const cv::Mat& mat);

private:
    volcart::gui::FetchSliceThread fetchSliceThread_;
    CannyThread cannyThread_;

    cv::Mat originalSliceMat_;
};
