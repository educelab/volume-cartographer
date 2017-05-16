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
 * @warning Why would you use this class? Don't use this class.
 *
 * @ingroup stps
 */
class Particle : public cv::Vec3d
{
public:
    /** @brief Constructor with Stop status initialization */
    explicit Particle(cv::Vec3d p) : cv::Vec3d(p), isStopped_(false) {}

    /** @brief Return Stop status of particle */
    bool isStopped() const { return isStopped_; }

    /** @brief Set Stop status on particle */
    void stop() { isStopped_ = true; }

private:
    bool isStopped_;
};
}
}