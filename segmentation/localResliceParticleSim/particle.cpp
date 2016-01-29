#include "particle.h"

using namespace volcart::segmentation;

Particle::Particle(cv::Vec3d position) : position_(position), isStopped_(false) {
}

Particle::Particle(double x, double y, double z) : isStopped_(false) {
    position_ = cv::Vec3d(x, y, z);
}

// Position in 3D space (Slice, X, Y)
cv::Vec3d Particle::position() const {
    return position_;
}

// Returns true if particle is stopped
bool Particle::isMoving() const {
    return !isStopped_;
}

// Sets particle as being stopped
void Particle::stop() {
    isStopped_ = true;
}

Particle& Particle::operator+=(const Particle& rhs) {
    position_ += rhs.position();
    return *this;
}

Particle Particle::operator+(const Particle& rhs) const
{
    return Particle(*this) += rhs;
}

Particle& Particle::operator-=(const Particle& rhs) {
    position_ -= rhs.position();
    return *this;
}

Particle Particle::operator-(const Particle& rhs) const {
    return Particle(*this) -= rhs;
}

double Particle::operator()(const int index) const {
    return position_(index);
}
