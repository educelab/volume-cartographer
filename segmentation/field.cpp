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
  dx = modf(point(VC_INDEX_X), &int_part);
  dy = modf(point(VC_INDEX_Y), &int_part);
  dz = modf(point(VC_INDEX_Z), &int_part);

  int x_min, x_max, y_min, y_max, z_min, z_max;
  x_min = (int)point(VC_INDEX_X);
  x_max = x_min + 1;
  y_min = (int)point(VC_INDEX_Y);
  y_max = y_min + 1;
  z_min = (int)point(VC_INDEX_Z);
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

Slice Field::reslice(cv::Vec3f center, cv::Vec3f x_dir, cv::Vec3f y_dir, int reslice_height, int reslice_width) {
  cv::Vec3f x_direction = normalize(x_dir);
  cv::Vec3f y_direction = normalize(y_dir);

  cv::Mat m(reslice_height, reslice_width, CV_16UC1);
  cv::Vec3f output_origin = center - ((reslice_width / 2) * x_direction + (reslice_height / 2) * y_direction);

  for (int height = 0; height < reslice_height; ++height) {
    for (int width = 0; width < reslice_width; ++width) {
      cv::Vec3f v = output_origin + (height * y_direction) + (width * x_direction);
      m.at<unsigned short>(height, width) = this->interpolate_at(v);
    }
  }

  return Slice(m, output_origin, x_direction, y_direction);
}
