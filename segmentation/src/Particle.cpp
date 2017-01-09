#include "segmentation/stps/Particle.h"

Particle::Particle(cv::Vec3d position)
{
    _position = position;
    _isStopped = false;
}

// Position in 3D space (Slice, X, Y)
cv::Vec3d Particle::position() { return _position; }

// Returns true if particle is stopped
bool Particle::isStopped() { return _isStopped; }

// Sets particle as being stopped
void Particle::stop() { _isStopped = true; }

// Component wise operators
void Particle::operator+=(cv::Vec3d v) { _position += v; }

double Particle::operator()(int index) { return _position(index); }

cv::Vec3d Particle::operator-(Particle p) { return _position - p.position(); }
