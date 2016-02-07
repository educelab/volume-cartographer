#include "particle.h"

// Position in 3D space (Slice, X, Y)
cv::Vec3d Particle::position() const { return _position; }
// Returns true if particle is stopped
bool Particle::isStopped() const { return _isStopped; }
// Sets particle as being stopped
void Particle::stop() { _isStopped = true; }
