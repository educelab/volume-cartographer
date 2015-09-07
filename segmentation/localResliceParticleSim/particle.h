#pragma once

#include <opencv2/opencv.hpp>

namespace volcart {

namespace segmentation {

class Particle {
public:
    Particle(cv::Vec3f);

    Particle(float, float, float);

    cv::Vec3f position();

    bool isStopped();

    void stop();

    void operator+=(cv::Vec3f);

    float operator()(int);

    cv::Vec3f operator-(Particle&);

private:
    cv::Vec3f _position;
    bool _isStopped;
};

}

}
