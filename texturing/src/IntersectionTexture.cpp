//
// Created by Seth Parker on 5/12/17.
//

#include "vc/texturing/IntersectionTexture.hpp"

using namespace volcart;
using namespace volcart::texturing;

Texture IntersectionTexture::compute()
{
    // Setup
    result_ = Texture();
    auto height = static_cast<int>(ppm_.height());
    auto width = static_cast<int>(ppm_.width());

    // Output image
    cv::Mat image = cv::Mat::zeros(height, width, CV_64FC1);

    // Iterate through the mappings
    auto mappings = ppm_.getSortedMappings();
    for (const auto& pixel : mappings) {
        // Assign the intensity value at the XY position
        image.at<uint16_t>(pixel.y, pixel.x) =
            vol_->interpolatedIntensityAt(pixel.pos);
    }

    // Set output
    result_.addImage(image);
    result_.setPPM(ppm_);

    return result_;
}