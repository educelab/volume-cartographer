#include "vc/texturing/LayerTexture.hpp"

#include <opencv2/core.hpp>

using namespace volcart;
using namespace volcart::texturing;

using Texture = LayerTexture::Texture;

Texture LayerTexture::compute()
{
    // Setup
    result_.clear();
    auto height = static_cast<int>(ppm_->height());
    auto width = static_cast<int>(ppm_->width());

    // Setup output images
    for (size_t i = 0; i < gen_->extents()[0]; i++) {
        result_.emplace_back(cv::Mat::zeros(height, width, CV_16UC1));
    }

    // Get the mappings
    auto mappings = ppm_->getMappings();

    // Sort the mappings by Z-value
    std::sort(
        mappings.begin(), mappings.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.pos[2] < rhs.pos[2];
        });

    // Iterate through the mappings
    auto counter = 0;
    auto updateStepSize =
        static_cast<int>(std::max(mappings.size() / 10000, 1UL));
    progressStarted();

    for (const auto& pixel : mappings) {
        if (counter++ % updateStepSize == 0) {
            progressUpdated(counter);
        }

        // Generate the neighborhood
        auto neighborhood = gen_->compute(vol_, pixel.pos, {pixel.normal});

        // Assign to the output images
        size_t it = 0;
        for (const auto& v : neighborhood) {
            result_.at(it++).at<uint16_t>(
                static_cast<int>(pixel.y), static_cast<int>(pixel.x)) = v;
        }
    }

    progressComplete();

    return result_;
}