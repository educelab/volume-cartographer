// What am I?
#pragma once

#include <opencv2/core.hpp>

typedef bool ParticleStopped;

class Particle
{
public:
    Particle(cv::Vec3d);
    cv::Vec3d position();
    bool isStopped();
    void stop();

    void operator+=(cv::Vec3d);
    float operator()(int);
    cv::Vec3d operator-(Particle);

private:
    cv::Vec3d _position;
    bool _isStopped;
};
