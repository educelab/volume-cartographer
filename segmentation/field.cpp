#include "field.h"
// Constructor
Field::Field(VolumePkg* v) {
  _volpkg = v;
  for (int i = 0; i < _volpkg->getNumberOfSlices(); ++i) {
    _field.push_back(v->getSliceAtIndex(i));
  }
}

// Trilinear Interpolation: Particles are not required
// to be at integer positions so we estimate their
// normals with their neighbors's known normals.
//
// formula from http://paulbourke.net/miscellaneous/interpolation/
unsigned short Field::interpolate_at(cv::Vec3f point) {
  double dx, dy, dz, int_part;
  dx = modf(point(0), &int_part);
  dy = modf(point(1), &int_part);
  dz = modf(point(2), &int_part);

  int x_min, x_max, y_min, y_max, z_min, z_max;
  x_min = (int)point(0);
  x_max = x_min + 1;
  y_min = (int)point(1);
  y_max = y_min + 1;
  z_min = (int)point(2);
  z_max = z_min + 1;

  // insert safety net
  if (x_min < 0 || y_min < 0 || z_min < 0)
    return 0;
  if (x_max >= _field[0].cols || y_max >= _field[0].rows)
    return 0;
  if (z_max >= _field.size())
    return 0;

  unsigned short vector =
    _field[z_min].at<unsigned short>(y_min, x_min) * (1 - dx) * (1 - dy) * (1 - dz) +
    _field[z_max].at<unsigned short>(y_min, x_min) * dx       * (1 - dy) * (1 - dz) +
    _field[z_min].at<unsigned short>(y_max, x_min) * (1 - dx) * dy       * (1 - dz) +
    _field[z_min].at<unsigned short>(y_min, x_max) * (1 - dx) * (1 - dy) * dz +
    _field[z_max].at<unsigned short>(y_min, x_max) * dx       * (1 - dy) * dz +
    _field[z_min].at<unsigned short>(y_max, x_max) * (1 - dx) * dy       * dz +
    _field[z_max].at<unsigned short>(y_max, x_min) * dx       * dy       * (1 - dz) +
    _field[z_max].at<unsigned short>(y_max, x_max) * dx       * dy       * dz;

  return vector;
}

#ifndef SLICE_DIR
#define SLICE_DIR cv::Vec3f(0,0,1)
#endif

cv::Mat Field::reslice(cv::Vec3f center, cv::Vec3f n, int reslice_height, int reslice_width) {
  cv::Vec3f normal = normalize(n);

  cv::Mat m(reslice_height, reslice_width, CV_16UC1);
  cv::Vec3f output_origin = center - ((reslice_width / 2) * normal + (reslice_height / 2) * SLICE_DIR);

  for (int height = 0; height < reslice_height; ++height) {
    for (int width = 0; width < reslice_width; ++width) {
      cv::Vec3f v = output_origin + (height * SLICE_DIR) + (width * normal);
      m.at<unsigned short>(height, width) = this->interpolate_at(v);
    }
  }

  return m;
}
