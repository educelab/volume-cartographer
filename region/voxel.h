#ifndef _VOXEL_
#define _VOXEL_

#include <iostream>
#include <opencv2/opencv.hpp>

typedef cv::Vec3f Vector;

class Voxel {
 public:
  Voxel(Vector, Vector, float, unsigned short);
  void project();

  Vector pos();
  Vector norm();
  Vector grav();
  Vector slice();
  float packedColor();

  friend bool operator<(const Voxel&, const Voxel&);
  friend Vector offset(Voxel, Voxel);
  friend std::ostream& operator<<(std::ostream& os, const Voxel& v);

 private:
  Vector position;
  Vector normal;
  Vector gravity;
  Vector slice_intersect;
  float eigen;

  unsigned short color;
};

#endif // _VOXEL_
