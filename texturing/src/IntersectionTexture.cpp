//
// Created by Seth Parker on 5/12/17.
//

#include "vc/texturing/IntersectionTexture.hpp"

#include <algorithm>

using namespace volcart;
using namespace volcart::texturing;

Texture IntersectionTexture::compute()
{
    // Setup
    result_ = Texture();
    auto height = static_cast<int>(ppm_.height());
    auto width = static_cast<int>(ppm_.width());

    // Output image
    cv::Mat image = cv::Mat::zeros(height, width, CV_16UC1);

    // Get the mappings
    auto mappings = ppm_.getMappings();

    // Sort the mappings by Z-value
    std::sort(
        mappings.begin(), mappings.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.pos[2] < rhs.pos[2];
        });

    // Iterate through the mappings
    for (const auto& pixel : mappings) {
        // Assign the intensity value at the XY position
        image.at<uint16_t>(
            static_cast<int>(pixel.y), static_cast<int>(pixel.x)) =
            vol_->interpolateAt(pixel.pos);
    }

    // Set output
    result_.addImage(image);
    result_.setPPM(ppm_);

    return result_;
}