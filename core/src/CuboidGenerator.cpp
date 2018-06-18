#include "vc/core/neighborhood/CuboidGenerator.hpp"

#include <exception>

static const std::vector<cv::Vec3d> BASIS_VECTORS = {
    {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

using namespace volcart;

Neighborhood CuboidGenerator::compute(
    Volume::Pointer v, cv::Vec3d pt, std::vector<cv::Vec3d> axes)
{
    // Auto-generate missing axes
    if (autoGenAxes_) {
        if (axes.size() == 1) {
            // Find a basis vector not parallel to n
            cv::Vec3d basis;
            for (const auto& b : BASIS_VECTORS) {
                if (axes[0].dot(b) != 1.0) {
                    basis = b;
                    break;
                }
            }
            axes.emplace_back(cv::normalize(axes[0].cross(basis)));
        }

        if (axes.size() == 2) {
            axes.emplace_back(cv::normalize(axes[0].cross(axes[1])));
        }
    }

    // If we don't have enough axes by this point, we're doing it wrong
    if (axes.size() < 3) {
        auto msg = "Invalid number of axes (" + std::to_string(axes.size()) +
                   "). Need 3.";
        throw std::invalid_argument(msg);
    }

    // Get center and primary radius of directional subvolume
    auto center = pt;
    auto radius = radius_;
    if (direction_ != Direction::Bidirectional) {
        radius[0] /= 2.0;
        auto offset = axes[0] * radius[0];
        if (direction_ == Direction::Negative) {
            offset *= -1;
        }
        center += offset;
    }

    // Get the number of samples along each basis
    auto extent = extents();

    // Iterate over the axes
    Neighborhood output(3, extent);
    for (size_t z = 0; z < extent[0]; ++z) {
        for (size_t y = 0; y < extent[1]; ++y) {
            for (size_t x = 0; x < extent[2]; ++x) {

                // Offset along each axis
                auto zOffset = -radius[0] + (z * interval_);
                auto yOffset = -radius[1] + (y * interval_);
                auto xOffset = -radius[2] + (x * interval_);

                // Current 3D position
                auto p = center + (axes[2] * xOffset) + (axes[1] * yOffset) +
                         (axes[0] * zOffset);

                // Assign to the subvolume array
                output(z, y, x) = v->interpolateAt(p);
            }
        }
    }

    return output;
}

Neighborhood::Extent CuboidGenerator::extents() const
{
    auto radius =
        (direction_ != Direction::Bidirectional) ? radius_[0] / 2 : radius_[0];

    Neighborhood::Extent extent;
    extent.emplace_back(
        static_cast<size_t>(std::floor(2.0 * radius / interval_) + 1));
    extent.emplace_back(
        static_cast<size_t>(std::floor(2.0 * radius_[1] / interval_) + 1));
    extent.emplace_back(
        static_cast<size_t>(std::floor(2.0 * radius_[2] / interval_) + 1));

    return extent;
}