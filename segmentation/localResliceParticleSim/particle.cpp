#include "particle.h"

using namespace volcart::segmentation;

Particle::Particle(cv::Vec3f position) : _position(position), _isStopped(false) {
}

Particle::Particle(float x, float y, float z) : _isStopped(false) {
    _position = cv::Vec3f(x, y, z);
}

// Position in 3D space (Slice, X, Y)
cv::Vec3f Particle::position() const {
    return _position;
}

// Returns true if particle is stopped
bool Particle::isMoving() const {
    return !_isStopped;
}

// Sets particle as being stopped
void Particle::stop() {
    _isStopped = true;
}

// Component wise operators
void Particle::operator+=(cv::Vec3f v) {
    _position += v;
}

float Particle::operator()(int index) const {
    return _position(index);
}

cv::Vec3f Particle::operator-(Particle& p) {
    return cv::Vec3f(_position - p.position());
}