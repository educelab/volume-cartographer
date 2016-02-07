#pragma once
// What am I?

#ifndef _PARTICLE_
#define _PARTICLE_

#include <opencv2/core/core.hpp>

typedef bool ParticleStopped;

class Particle : cv::Vec3d
{
public:
    Particle(const cv::Vec3d v) : _position(v), _isStopped(false) {}
    cv::Vec3d position() const { return _position; }
    bool isStopped() const { return _isStopped; }
    void stop() { _isStopped = true; };
private:
    cv::Vec3d _position;
    bool _isStopped;
};

#endif
