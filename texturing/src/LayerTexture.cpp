#include "vc/texturing/LayerTexture.hpp"

#include <opencv2/core.hpp>

#include "vc/core/util/Iteration.hpp"

using namespace volcart;
using namespace volcart::texturing;

using Texture = LayerTexture::Texture;

auto LayerTexture::compute() -> Texture
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
    progressStarted();
    for (const auto [idx, pixel] : enumerate(mappings)) {
        progressUpdated(idx);

        // Generate the neighborhood
        auto neighborhood = gen_->compute(vol_, pixel.pos, {pixel.normal});

        // Assign to the output images
        for (const auto [it, v] : enumerate(neighborhood)) {
            const auto yy = static_cast<int>(pixel.y);
            const auto xx = static_cast<int>(pixel.x);
            result_.at(it).at<std::uint16_t>(yy, xx) = v;
        }
    }
    progressComplete();

    return result_;
}