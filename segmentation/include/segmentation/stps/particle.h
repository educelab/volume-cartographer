// What am I?
#pragma once

#include <opencv2/opencv.hpp>

using ParticleStopped = bool;

class Particle
{
public:
    Particle(cv::Vec3d position);
    cv::Vec3d position();
    bool isStopped();
    void stop();

    void operator+=(cv::Vec3d const& v);
    float operator()(int index);
    cv::Vec3d operator-(Particle p);

private:
    cv::Vec3d _position;
    bool _isStopped;
};
