#pragma once

#include "ImageScrollArea.hpp"
#include "PPMProjectionThread.hpp"

class PPMProjectionViewerWidget : public ImageScrollArea
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