#pragma once

/** @file */

#include <opencv2/core.hpp>

namespace volcart
{
namespace segmentation
{
/**
 * @class Particle
 * @brief A simple particle class
 *
 * Keeps track of particle position and the resting distance between particles.
 *
 * @ingroup stps
 */
class Particle
{
public:
    /** @brief Default constructor */
    Particle() = default;

    /** @brief Constructor with position initialization */
    explicit Particle(const cv::Vec3d& p) : pos_(p) {}

    /** @brief Add a vector offset to the particle position */
    Particle& operator+=(const cv::Vec3d& rhs);
    /** @brief Scale all elements of the particle position */
    Particle& operator*=(const double& rhs);

    /** @brief Get the particle position */
    cv::Vec3d& pos() { return pos_; }
    /** @copydoc pos() */
    const cv::Vec3d& pos() const { return pos_; }

    /**
     * @brief Get the resting length between this and the previous particle in
     * the chain
     */
    double& restingL() { return restingL_; }
    /** @copydoc restingL() */
    const double& restingL() const { return restingL_; }

    /**
     * @brief Get the resting length between this and the next particle in the
     * chain
     */
    double& restingR() { return restingR_; }
    /** @copydoc restingR() */
    const double& restingR() const { return restingR_; }

private:
    /** Particle position */
    cv::Vec3d pos_{0, 0, 0};
    /** Resting length to the "left" */
    double restingL_{0};
    /** Resting length to the "right" */
    double restingR_{0};
};

/** Free function operator for Particle and vector offset addition */
Particle operator+(Particle lhs, const cv::Vec3d& rhs);
/** Free function operator for Particle and scale factor multiplication */
Particle operator*(Particle lhs, const double& rhs);
}  // namespace segmentation
}  // namespace volcart
