#pragma once
// What am I?

#ifndef _PARTICLE_
#define _PARTICLE_

#include <opencv2/core/core.hpp>

typedef bool ParticleStopped;

class Particle : public cv::Vec3d
{
public:
    Particle(const double x, const double y, const double z) : _isMoving(true)
    {
    }
    Particle(const cv::Vec3d v) : _isMoving(true) {}
    cv::Vec3d position() const { return {(0), (1), (2)}; }
    bool isMoving() const { return _isMoving; }
    void stop() { _isMoving = false; };
private:
    bool _isMoving;
};

#endif
