//
// Created by Seth Parker on 5/12/17.
//

#include "vc/texturing/CompositeTexture.hpp"

#include <algorithm>

#include "vc/core/util/FloatComparison.hpp"

static constexpr double MEDIAN_MEAN_PERCENT_RANGE = 0.70;

using namespace volcart;
using namespace volcart::texturing;

Texture CompositeTexture::compute()
{
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
            auto pixelInfo = ppm_(y, x);
            cv::Vec3d xyz{pixelInfo[0], pixelInfo[1], pixelInfo[2]};
            cv::Vec3d xyzNorm{pixelInfo[3], pixelInfo[4], pixelInfo[5]};

            // Generate the neighborhood
            auto neighborhood = vol_->getVoxelNeighborsLinearInterpolated(
                xyz, xyzNorm, radius_, interval_, direction_);

            // Assign the intensity value at the UV position
            image.at<uint16_t>(y, x) = filter_neighborhood_(neighborhood);
        }
    }

    // Set output
    result_.addImage(image);
    result_.setPPM(ppm_);

    return result_;
}

uint16_t CompositeTexture::filter_neighborhood_(const Neighborhood& n)
{
    switch (filter_) {
        case Filter::Minimum:
            return min_(n);
        case Filter::Maximum:
            return max_(n);
        case Filter::Median:
            return median_(n);
        case Filter::Mean:
            return mean_(n);
        case Filter::MedianAverage:
            return median_mean_(n, MEDIAN_MEAN_PERCENT_RANGE);
    }
}

uint16_t CompositeTexture::min_(Neighborhood n)
{
    return *std::min_element(n.begin(), n.end());
}

uint16_t CompositeTexture::max_(Neighborhood n)
{
    return *std::max_element(n.begin(), n.end());
}

uint16_t CompositeTexture::median_(Neighborhood n)
{
    std::sort(n.begin(), n.end());
    return n[n.size() / 2];
}

uint16_t CompositeTexture::mean_(Neighborhood n)
{
    double res = 0.0;
    for (auto& v : n) {
        res += v;
    }
    return static_cast<uint16_t>(std::round(res / n.size()));
}

uint16_t CompositeTexture::median_mean_(Neighborhood n, double range)
{
    // If the range is 1.0, it's just a normal mean operation
    if (AlmostEqual<double>(range, 1.0)) {
        return mean_(n);
    } else if (AlmostEqual<double>(range, 0.0)) {
        return 0;
    }

    // Sort
    std::sort(n.begin(), n.end());

    // The number of things we're going to sum
    auto count = static_cast<size_t>(std::ceil(n.size() * range));
    // The number of things before we start summing
    auto offset = static_cast<size_t>(std::floor((n.size() - count) / 2.0));

    // Sum
    double res = 0.0;
    for (auto it = offset; it < offset + count; it++) {
        res += n.at(it);
    }

    // Average
    return static_cast<uint16_t>(std::round(res / count));
}