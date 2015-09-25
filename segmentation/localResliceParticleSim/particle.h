#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

namespace volcart {

namespace segmentation {

class Particle {
public:
    Particle(cv::Vec3f);

    Particle(float, float, float);

    cv::Vec3f position() const;

    bool isMoving() const;

    void stop();

    void operator+=(cv::Vec3f);

    float operator()(int) const;

    cv::Vec3f operator-(Particle&);

private:
    friend std::ostream& operator<<(std::ostream& s, Particle& p) {
        return s << p._position;
    }

    cv::Vec3f _position;
    bool _isStopped;
};

}

}
