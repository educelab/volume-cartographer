#include "particle.h"

Particle::Particle(cv::Vec3f position) {
  _position = position;
  _status = false;
}

// Position in 3D space (Slice, X, Y)
cv::Vec3f Particle::position() {
  return _position;
}

// Returns true if particle is stopped //*To-Do: Should return false if particle is stopped. Typedef it son.
bool Particle::status() {
  return _status;
}

// Sets particle as being stopped
void Particle::invalidate() { //To-Do: Change to stop()
  _status = true;
}

// Component wise operators
void Particle::operator+=(cv::Vec3f v) {
  _position += v;
}

float Particle::operator()(int index) {
  return _position(index);
}

cv::Vec3f Particle::operator-(Particle p) {
  return _position - p.position();
}
