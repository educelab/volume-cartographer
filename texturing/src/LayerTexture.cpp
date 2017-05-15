//
// Created by Seth on 11/24/16.
//
#include "vc/texturing/LayerTexture.hpp"

#include <opencv2/core.hpp>

using namespace volcart;
using namespace volcart::texturing;

Texture LayerTexture::compute()
{
    // Setup
    result_ = Texture();
    auto height = static_cast<int>(ppm_.height());
    auto width = static_cast<int>(ppm_.width());

    // Setup output images
    for (size_t i = 0; i < neighborhood_count_(); i++) {
        result_.addImage(cv::Mat::zeros(height, width, CV_16UC1));
    }

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Skip this pixel if we have no mapping
            if (!ppm_.hasMapping(y, x)) {
                continue;
            }

            // Find the xyz coordinate of the original point
            auto pixelInfo = ppm_(y, x);
            cv::Vec3d xyz{pixelInfo[0], pixelInfo[1], pixelInfo[2]};
            cv::Vec3d xyzNorm{pixelInfo[3], pixelInfo[4], pixelInfo[5]};

            // Generate the neighborhood
            auto neighborhood = vol_->getVoxelNeighborsLinearInterpolated(
                xyz, xyzNorm, radius_, interval_, direction_);

            // Assign to the output images
            for (size_t i = 0; i < neighborhood.size(); i++) {
                result_.image(i).at<uint16_t>(y, x) = neighborhood[i];
            }
        }
    }

    result_.setPPM(ppm_);

    return result_;
}

size_t LayerTexture::neighborhood_count_()
{
    auto radius = std::abs(radius_);

    // Setup Range
    double min, max;
    switch (direction_) {
        case Direction::Bidirectional: {
            min = -1 * radius;
            max = radius;
            break;
        }
        case Direction::Positive: {
            min = 0;
            max = radius;
            break;
        }
        case Direction::Negative: {
            min = -1 * radius;
            max = 0;
            break;
        }
    }

    return static_cast<size_t>(std::floor((max - min) / interval_) + 1);
}