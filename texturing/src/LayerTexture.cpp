#include "vc/texturing/LayerTexture.hpp"

#include <opencv2/core.hpp>

#include "vc/core/util/Iteration.hpp"

using namespace volcart;
using namespace volcart::texturing;

using Texture = LayerTexture::Texture;

auto LayerTexture::New() -> Pointer { return std::make_shared<LayerTexture>(); }

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
    auto mappings = ppm_->getMappingCoords();

    // Sort the mappings by Z-value
    std::sort(
        mappings.begin(), mappings.end(),
        [&](const auto& lhs, const auto& rhs) {
            return (*ppm_)(lhs.y, lhs.x)[2] < (*ppm_)(rhs.y, rhs.x)[2];
        });

    // Iterate through the mappings
    // TODO: Test enumerate performance
    progressStarted();
    for (const auto [idx, coord] : enumerate(mappings)) {
        progressUpdated(idx);

        // Generate the neighborhood
        const auto [y, x] = coord;
        const auto& m = ppm_->getMapping(y, x);
        const cv::Vec3d pos{m[0], m[1], m[2]};
        const cv::Vec3d normal{m[3], m[4], m[5]};
        auto neighborhood = gen_->compute(vol_, pos, {normal});

        // Assign to the output images
        for (const auto [it, v] : enumerate(neighborhood)) {
            const auto yy = static_cast<int>(y);
            const auto xx = static_cast<int>(x);
            result_.at(it).at<std::uint16_t>(yy, xx) = v;
        }
    }
    progressComplete();

    return result_;
}