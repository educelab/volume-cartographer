#pragma once

#ifndef _VOLCART_SEGMENTATION_PARTICLE_H_
#define _VOLCART_SEGMENTATION_PARTICLE_H_

#include <iostream>
#include <opencv2/opencv.hpp>

#include "common.h"

namespace volcart {

namespace segmentation {

class Particle {
public:
    Particle(cv::Vec3f);

    Particle(float, float, float);

    cv::Vec3f position() const;

    bool isMoving() const;

    void stop();

    float operator()(int) const;

    Particle& operator+=(const Particle&);

    Particle& operator-=(const Particle&);

    Particle operator+(const Particle& rhs) const;

    Particle operator-(const Particle& rhs) const;

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
