// What am I?
#pragma once

#include <opencv2/core.hpp>

class Particle
{
public:
    explicit Particle(cv::Vec3d position)
        : position_{position}, isStopped_{false}
    {
    }

    cv::Vec3d position() const { return position_; }
    bool isStopped() const { return isStopped_; }
    void stop() { isStopped_ = true; }

    void operator+=(const cv::Vec3d& v) { position_ += v; }
    double operator()(int index) const { return position_(index); }
    cv::Vec3d operator-(const Particle& p) { return position_ - p.position(); }

private:
    cv::Vec3d position_;
    bool isStopped_;
};
