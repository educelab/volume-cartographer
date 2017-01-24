// What am I?
#pragma once

#include <opencv2/core.hpp>

/**
 * @class Particle
 * @brief Defines how a particle moves
 * @ingroup stps
 */
class Particle
{
public:
    /**@brief Initializes a particle to have a specific position */
    explicit Particle(cv::Vec3d position)
        : position_{position}, isStopped_{false}
    {
    }

    /** @brief Returns particle position in 3D space */
    cv::Vec3d position() const { return position_; }

    /** @brief Returns True if particle is stopped*/
    bool isStopped() const { return isStopped_; }

    /** @brief Sets the particle as stopped */
    void stop() { isStopped_ = true; }

    /** @brief Overwrites operator to add to the position
    *
    * Overwrites the += operator so that it adds every
    * component of the parameter vec to the position
    * vector of the particle.
    */
    void operator+=(const cv::Vec3d& v) { position_ += v; }

    /** @brief Sets the () operator to return the component stored at int */
    double operator()(int index) const { return position_(index); }

    /** @brief Subracts the position of the Particle from the position of the
     * current particle */
    cv::Vec3d operator-(const Particle& p) { return position_ - p.position(); }

private:
    /** Current position of the particle*/
    cv::Vec3d position_;
    /** Determines if the particle is still moving*/
    bool isStopped_;
};
