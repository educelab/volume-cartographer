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
    for (size_t i = 0; i < gen_->extents()[0]; i++) {
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
        auto neighborhood = gen_->compute(vol_, pixel.pos, {pixel.normal});

        // Assign to the output images
        size_t it = 0;
        for (const auto& v : neighborhood) {
            result_.image(it++).at<uint16_t>(
                static_cast<int>(pixel.y), static_cast<int>(pixel.x)) = v;
        }
    }

    result_.setPPM(ppm_);

    return result_;
}