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

    cv::Mat image = cv::Mat::zeros(height, width, CV_16UC1);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Skip this pixel if we have no mapping
            if (!ppm_.hasMapping(y, x)) {
                continue;
            }

            // Find the xyz coordinate of the original point
            auto p = ppm_(y, x);

            // Assign the intensity value at the XY position
            image.at<uint16_t>(y, x) =
                vol_->interpolatedIntensityAt(p[0], p[1], p[2]);
        }
    }

    // Set output
    result_.addImage(image);
    result_.setPPM(ppm_);

    return result_;
}