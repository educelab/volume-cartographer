#include "vc/texturing/ThicknessTexture.hpp"

#include "vc/core/util/Iteration.hpp"

using namespace volcart;
using namespace volcart::texturing;

void ThicknessTexture::setSamplingInterval(double i) { interval_ = i; }

void ThicknessTexture::setNormalizeOutput(bool b) { normalize_ = b; }

void ThicknessTexture::setVolumetricMask(const VolumetricMask::Pointer& m)
{
    mask_ = m;
}

Texture ThicknessTexture::compute()
{
    // Setup
    result_ = Texture();
    auto height = static_cast<int>(ppm_.height());
    auto width = static_cast<int>(ppm_.width());

    // Output image
    cv::Mat image = cv::Mat::zeros(height, width, CV_32FC1);

    // Get the mappings
    auto mappings = ppm_.getMappings();

    // Sort the mappings by Z-value
    std::sort(
        mappings.begin(), mappings.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.pos[2] < rhs.pos[2];
        });

    // Iterate through the mappings
    progressStarted();
    for (const auto it : enumerate(mappings)) {
        progressUpdated(it.first);
        const auto& pixel = it.second;

        // Starting voxel must be in mask
        if (mask_->isIn(pixel.pos)) {
            // Setup bidirectional search
            bool foundMin{false};
            bool foundMax{false};
            cv::Vec3d min{pixel.pos};
            cv::Vec3d max{pixel.pos};
            double offset{0};

            // Find the edges of the layer from this point
            while (!foundMin || !foundMax) {
                // Calculate offset
                offset += interval_;
                auto delta = offset * pixel.normal;

                // Check the negative direction
                if (!foundMin) {
                    auto neg = pixel.pos - delta;
                    foundMin = mask_->isOut(neg);
                    min = (foundMin) ? min : neg;
                }

                // Check the positive direction
                if (!foundMax) {
                    auto pos = pixel.pos + delta;
                    foundMax = mask_->isOut(pos);
                    max = (foundMax) ? max : pos;
                }
            }

            // Assign the intensity value at the UV position
            auto x = static_cast<int>(pixel.x);
            auto y = static_cast<int>(pixel.y);

            // If max = min, then thickness 1
            // Otherwise, thickness == distance
            if (max == min) {
                image.at<float>(y, x) = 1;
            } else {
                auto dist = cv::norm(max, min, cv::NORM_L2);
                image.at<float>(y, x) = static_cast<float>(dist);
            }
        }
    }
    progressComplete();

    if (normalize_) {
        cv::normalize(image, image, 0.0, 1.0, cv::NORM_MINMAX);
    }

    // Set output
    result_.addImage(image);
    result_.setPPM(ppm_);

    return result_;
}