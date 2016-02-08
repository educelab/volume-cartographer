#pragma once
// What am I?

#ifndef _PARTICLE_
#define _PARTICLE_

#include <opencv2/core/core.hpp>

typedef bool ParticleStopped;

class Particle : public cv::Vec3d
{
public:
    Particle(const double x, const double y, const double z) : _isStopped(false)
    {
    }
    Particle(const cv::Vec3d v) : _isStopped(false) {}
    cv::Vec3d position() const { return {(0), (1), (2)}; }
    bool isStopped() const { return _isStopped; }
    void stop() { _isStopped = true; };
private:
    bool _isStopped;
};

#endif
