#include "particle.h"

Particle::Particle(cv::Vec3f position) {
  _position = position;
  _status = false;
}

cv::Vec3f Particle::position() {
  return _position;
}

bool Particle::status() {
  return _status;
}

void Particle::invalidate() {
  _status = true;
}

void Particle::operator+=(cv::Vec3f v) {
  _position += v;
}

float Particle::operator()(int index) {
  return _position(index);
}

cv::Vec3f Particle::operator-(Particle p) {
  return _position - p.position();
}
