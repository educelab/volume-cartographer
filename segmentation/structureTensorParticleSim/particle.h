// What am I?

#ifndef _PARTICLE_
#define _PARTICLE_

#include <opencv2/opencv.hpp>

typedef bool ParticleStopped;

class Particle {
 public:
  Particle(cv::Vec3f);
  cv::Vec3f position();
  bool isStopped();
  void stop();

  void operator+=(cv::Vec3f);
  float operator()(int);
  cv::Vec3f operator-(Particle);

 private:
  cv::Vec3f _position;
  bool _isStopped;
};

#endif
