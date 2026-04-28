#include "vc/texturing/LayerTexture.hpp"

#include <cstddef>

#include <opencv2/core.hpp>

#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;
using namespace volcart::texturing;

auto LayerTexture::New() -> Pointer { return std::make_shared<LayerTexture>(); }

void LayerTexture::setGenerator(LineGenerator::Pointer g)
{
    gen_ = std::move(g);
}

void LayerTexture::setEagerMode(const bool enable) { eager_ = enable; }

auto LayerTexture::getEagerMode() const -> bool { return eager_; }

auto LayerTexture::compute() -> Texture
{
    // Setup
    result_.clear();
    std::size_t numCompleted{0};
    const auto height = static_cast<int>(ppm_->height());
    const auto width = static_cast<int>(ppm_->width());

    // Get the mappings
    auto mappings = ppm_->getMappingCoords();

    // Sort the mappings by Z-value
    std::sort(
        mappings.begin(), mappings.end(),
        [&](const auto& lhs, const auto& rhs) {
            return (*ppm_)(lhs.y, lhs.x)[2] < (*ppm_)(rhs.y, rhs.x)[2];
        });

    // Generate the images
    progressStarted();
    // Eager mode: Iterate the layer stack
    if (eager_) {
        Logger()->debug("[LayerTexture] Starting layer generation (eager)");
        const auto offsets = gen_->offsets();
        // Iterate over the output offsets
        for (auto [it, offset] : enumerate(offsets)) {
            cv::Mat result = cv::Mat::zeros(height, width, CV_16UC1);
            // Iterate over the pixels
            for (const auto [idx, coord] : enumerate(mappings)) {
                progressUpdated((idx + it * mappings.size()) / offsets.size());
                // Get the mapping position
                const auto [y, x] = coord;
                const auto& m = ppm_->getMapping(y, x);
                const cv::Vec3d pos{m[0], m[1], m[2]};
                const cv::Vec3d normal{m[3], m[4], m[5]};

                // Interpolate the pixel
                const auto yy = static_cast<int>(y);
                const auto xx = static_cast<int>(x);
                const auto v = vol_->interpolateAt(pos + normal * offset);
                result.at<std::uint16_t>(yy, xx) = v;
            }

            // Notify that the image is complete
            numCompleted += 1;
            imageComplete.send(it, offsets.size(), result);
            if (eagerCache_) {
                result_.emplace_back(result);
            }
        }
    }
    // Regular mode: Iterate the pixel stack
    else {
        Logger()->debug("[LayerTexture] Starting layer generation");
        // Preallocate output images
        for (std::size_t i = 0; i < gen_->extents()[0]; i++) {
            result_.emplace_back(cv::Mat::zeros(height, width, CV_16UC1));
        }
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
        numCompleted = result_.size();
    }
    progressComplete();
    Logger()->debug("[LayerTexture] Generated {} layers", numCompleted);
    return result_;
}