#include "vc/core/neighborhood/LineGenerator.hpp"

#include <cstddef>

#include <educelab/core/utils/Iteration.hpp>

#include "vc/core/util/FloatComparison.hpp"

using namespace volcart;
namespace el = educelab;

namespace
{
auto GetOffsets(double radius, double interval, Direction direction)
    -> std::vector<double>
{
    // Make sure radius is positive
    radius = std::abs(radius);

    // Setup Range
    double min{}, max{};
    switch (direction) {
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
        default:
            min = -1 * radius;
            max = radius;
    }

    // Iterate through range
    const auto count =
        static_cast<std::size_t>(std::floor((max - min) / interval) + 1);
    std::vector<double> offsets;
    offsets.reserve(count);
    for (std::size_t it = 0; it < count; it++) {
        offsets.push_back(min + static_cast<double>(it) * interval);
    }
    return offsets;
}
}  // namespace

auto LineGenerator::compute(
    const Volume::Pointer& v,
    const cv::Vec3d& pt,
    const std::vector<cv::Vec3d>& axes) -> Neighborhood
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

    // Pregenerate the offsets
    auto offsets = GetOffsets(radius_[0], interval_, direction_);

    // Fill the neighborhood
    Neighborhood n(1, offsets.size());
    for (const auto [it, offset] : el::enumerate(offsets)) {
        n(it) = v->interpolateAt(pt + axes[0] * offset);
    }

    return n;
}

LineGenerator::LineGenerator() : NeighborhoodGenerator(1) {}

LineGenerator::Pointer LineGenerator::New()
{
    return std::make_shared<LineGenerator>();
}

auto LineGenerator::extents() const -> Neighborhood::Extent
{
    const auto r =
        direction_ != Direction::Bidirectional ? radius_[0] / 2 : radius_[0];
    return {static_cast<std::size_t>(std::floor(2.0 * r / interval_) + 1)};
}

auto LineGenerator::offsets() const -> std::vector<double>
{
    return GetOffsets(radius_[0], interval_, direction_);
}