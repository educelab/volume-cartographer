#pragma once

#include <opencv2/core.hpp>

namespace volcart
{
namespace segmentation
{
/**
 * @class Particle
 * @brief Extends cv::Vec3d with some other things for STPS
 *
 * @ingroup stps
 */
class Particle
{
public:
    Particle() = default;

    /** @brief Constructor */
    explicit Particle(const cv::Vec3d& p) : pos(p) {}

    Particle& operator+=(const cv::Vec3d& rhs);
    Particle& operator*=(const double& rhs);

    cv::Vec3d pos{0, 0, 0};

    double restingL{0};
    double restingR{0};
};

Particle operator+(Particle lhs, const cv::Vec3d& rhs);
Particle operator*(Particle lhs, const double& rhs);
}
}
