#include "vc/texturing/IntersectionTexture.hpp"

#include <algorithm>

using namespace volcart;
using namespace volcart::texturing;

using Texture = IntersectionTexture::Texture;

auto IntersectionTexture::New() -> Pointer
{
    return std::make_shared<IntersectionTexture>();
}

Texture IntersectionTexture::compute()
{
    // Setup
    result_.clear();
    auto height = static_cast<int>(ppm_->height());
    auto width = static_cast<int>(ppm_->width());

    // Output image
    cv::Mat image = cv::Mat::zeros(height, width, CV_16UC1);

    // Get the mappings
    auto mappings = ppm_->getMappingCoords();

    // Sort the mappings by Z-value
    std::sort(
        mappings.begin(), mappings.end(),
        [&](const auto& lhs, const auto& rhs) {
            return (*ppm_)(lhs.y, lhs.x)[2] < (*ppm_)(rhs.y, rhs.x)[2];
        });

    // Iterate through the mappings
    std::size_t counter{0};
    progressStarted();
    for (const auto [y, x] : mappings) {
        progressUpdated(counter++);

        // Assign the intensity value at the XY position
        const auto& m = ppm_->getMapping(y, x);
        image.at<uint16_t>(static_cast<int>(y), static_cast<int>(x)) =
            vol_->interpolateAt({m[0], m[1], m[2]});
    }
    progressComplete();

    // Set output
    result_.push_back(image);

    return result_;
}