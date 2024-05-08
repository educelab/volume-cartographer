#include "vc/texturing/ThicknessTexture.hpp"

#include "vc/core/util/Iteration.hpp"

using namespace volcart;
using namespace volcart::texturing;

using Texture = ThicknessTexture::Texture;

auto ThicknessTexture::New() -> Pointer
{
    return std::make_shared<ThicknessTexture>();
}

void ThicknessTexture::setSamplingInterval(double i) { interval_ = i; }

void ThicknessTexture::setNormalizeOutput(bool b) { normalize_ = b; }

void ThicknessTexture::setVolumetricMask(const VolumetricMask::Pointer& m)
{
    mask_ = m;
}

auto ThicknessTexture::compute() -> Texture
{
    // Setup
    result_.clear();
    auto height = static_cast<int>(ppm_->height());
    auto width = static_cast<int>(ppm_->width());

    // Output image
    cv::Mat image = cv::Mat::zeros(height, width, CV_32FC1);

    // Get the mappings
    auto mappings = ppm_->getMappingCoords();

    // Sort the mappings by Z-value
    std::sort(
        mappings.begin(), mappings.end(),
        [&](const auto& lhs, const auto& rhs) {
            return (*ppm_)(lhs.y, lhs.x)[2] < (*ppm_)(rhs.y, rhs.x)[2];
        });

    // Iterate through the mappings
    progressStarted();
    for (const auto [idx, coord] : enumerate(mappings)) {
        progressUpdated(idx);

        // Generate the neighborhood
        const auto [y, x] = coord;
        const auto& m = ppm_->getMapping(y, x);
        const cv::Vec3d pos{m[0], m[1], m[2]};
        const cv::Vec3d normal{m[3], m[4], m[5]};

        // Starting voxel must be in mask
        if (mask_->isIn(pos)) {
            // Setup bidirectional search
            bool foundMin{false};
            bool foundMax{false};
            cv::Vec3d min{pos};
            cv::Vec3d max{pos};
            double offset{0};

            // Find the edges of the layer from this point
            while (not foundMin or not foundMax) {
                // Calculate offset
                offset += interval_;
                auto delta = offset * normal;

                // Check the negative direction
                if (not foundMin) {
                    auto neg = pos - delta;
                    foundMin = mask_->isOut(neg);
                    min = (foundMin) ? min : neg;
                }

                // Check the positive direction
                if (not foundMax) {
                    auto newPos = pos + delta;
                    foundMax = mask_->isOut(newPos);
                    max = (foundMax) ? max : newPos;
                }
            }

            // Assign the intensity value at the UV position
            const auto u = static_cast<int>(x);
            const auto v = static_cast<int>(y);

            // If max = min, then thickness 1
            // Otherwise, thickness == distance
            if (max == min) {
                image.at<float>(v, u) = 1;
            } else {
                auto dist = cv::norm(max, min, cv::NORM_L2);
                image.at<float>(v, u) = static_cast<float>(dist);
            }
        }
    }
    progressComplete();

    if (normalize_) {
        cv::normalize(image, image, 0.0, 1.0, cv::NORM_MINMAX);
    }

    // Set output
    result_.push_back(image);

    return result_;
}

auto ThicknessTexture::samplingInterval() const -> double { return interval_; }

auto ThicknessTexture::normalizeOutput() const -> bool { return normalize_; }

auto ThicknessTexture::volumetricMask() const -> VolumetricMask::Pointer
{
    return mask_;
}
