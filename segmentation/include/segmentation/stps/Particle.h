// What am I?
#pragma once

#include <opencv2/core.hpp>

typedef bool ParticleStopped;
/**
 * @class Particle
 * @brief Defines how a particle moves
 * @ingroup stps
 */
class Particle
{
public:
    /**@brief Initializes a particle to have a specific position */
    Particle(cv::Vec3d);
    /** @brief Returns particle position in 3D space */
    cv::Vec3d position();
    /** @brief Returns True if particle is stopped*/
    bool isStopped();
    /** @brief Sets the particle as stopped */
    void stop();
    /** @brief Overwrites operator to add to the position
     *
    * Overwrites the += operator so that it adds every
    * component of the parameter vec to the position
    * vector of the particle.
    */
    void operator+=(cv::Vec3d);
    /** @brief Sets the () operator to return the component stored at int */
    double operator()(int);
    /** @brief Subracts the position of the Particle from the position of the
     * current particle*/
    cv::Vec3d operator-(Particle);

private:
    /** Current position of the particle*/
    cv::Vec3d _position;
    /** Determines if the particle is still moving*/
    bool _isStopped;
};
