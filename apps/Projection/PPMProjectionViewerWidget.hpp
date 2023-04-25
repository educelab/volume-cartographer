#pragma once

#include "PPMProjectionThread.hpp"
#include "vc/gui_support/ImageScrollArea.hpp"

class PPMProjectionViewerWidget : public volcart::gui::ImageScrollArea
{
    // clang-format off
    Q_OBJECT
    // clang-format on

public:
    PPMProjectionViewerWidget(
        const std::string& visualizePPMIntersection,
        const std::string& ppmImageOverlay);

public slots:
    void handleSliceChange(int sliceIdx);

private:
    PPMProjectionThread ppmProjectionThread_;
};