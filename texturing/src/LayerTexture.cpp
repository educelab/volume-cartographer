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

    // Get the mappings
    auto mappings = ppm_.getMappings();

    // Sort the mappings by Z-value
    std::sort(
        mappings.begin(), mappings.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.pos[2] < rhs.pos[2];
        });

    // Iterate through the mappings
    for (const auto& pixel : mappings) {
        // Generate the neighborhood
        auto neighborhood = vol_->getVoxelNeighborsLinearInterpolated(
            pixel.pos, pixel.normal, radius_, interval_, direction_);

        // Assign to the output images
        for (size_t i = 0; i < neighborhood.size(); i++) {
            result_.image(i).at<uint16_t>(
                static_cast<int>(pixel.y), static_cast<int>(pixel.x)) =
                neighborhood[i];
        }
    }

    result_.setPPM(ppm_);

    return result_;
}