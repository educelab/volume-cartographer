#pragma once

#ifndef _VOLCART_SEGMENTATION_PARTICLE_H_
#define _VOLCART_SEGMENTATION_PARTICLE_H_

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

    Particle& operator+=(const Particle&) const;

    float operator()(int) const;

    Particle& operator-=(const Particle&) const;

private:
    friend std::ostream& operator<<(std::ostream& s, Particle& p) {
        return s << p.position_;
    }

    cv::Vec3f position_;
    bool isStopped_;
};

}

}

#endif
