#pragma once
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-err58-cpp"
#include <cstdint>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

static const double MAX_8BPC = std::numeric_limits<std::uint8_t>::max();
static const double MAX_16BPC = std::numeric_limits<std::uint16_t>::max();

static const cv::Scalar WHITE{255, 255, 255};
static const cv::Scalar BLUE{255, 0, 0};
static const cv::Scalar GREEN{0, 255, 0};
static const cv::Scalar RED{0, 0, 255};

enum class Color { White = 0, Red, Green, Blue };

namespace volcart
{

struct ProjectionSettings {
    cv::Scalar color;
    int thickness = 1;
    bool intersectOnly = false;
    int zMin = 0;
    int zMax = 1;
    std::string visualizePPMIntersection;
    std::string ppmImageOverlay;
};

}  // namespace volcart
#pragma clang diagnostic pop
