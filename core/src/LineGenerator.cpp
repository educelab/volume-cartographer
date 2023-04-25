#include "vc/core/neighborhood/LineGenerator.hpp"

#include "vc/core/util/FloatComparison.hpp"

using namespace volcart;

Neighborhood LineGenerator::compute(
    const Volume::Pointer& v,
    const cv::Vec3d& pt,
    const std::vector<cv::Vec3d>& axes)
{
    // If we don't have enough axes by this point, we're doing it wrong
    if (axes.empty()) {
        auto msg = "Invalid number of axes (" + std::to_string(axes.size()) +
                   "). Need 1.";
        throw std::invalid_argument(msg);
    }

    // Interval bounds
    if (AlmostEqual(interval_, 0.0)) {
        throw std::domain_error("Sampling interval too small");
    }

    // Make sure radius is positive
    auto radius = std::abs(radius_[0]);

    // Setup Range
    double min, max;
    switch (direction_) {
        case Direction::Bidirectional: {
            min = -1 * radius;
            max = radius;
            break;
        }
        case Direction::Positive: {
            min = 0;
            max = radius;
            break;
        }
        case Direction::Negative: {
            min = -1 * radius;
            max = 0;
            break;
        }
    }

    // Iterate through range
    auto count = static_cast<size_t>(std::floor((max - min) / interval_) + 1);
    Neighborhood n(1, count);
    for (size_t it = 0; it < count; it++) {
        auto offset = min + (it * interval_);
        n(it) = v->interpolateAt(pt + (axes[0] * offset));
    }

    return n;
}

Neighborhood::Extent LineGenerator::extents() const
{
    auto radius =
        (direction_ != Direction::Bidirectional) ? radius_[0] / 2 : radius_[0];
    return {static_cast<size_t>(std::floor(2.0 * radius / interval_) + 1)};
}