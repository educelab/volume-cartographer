#include "field.h"

Field::Field(VolumePkg* v) {
  _volpkg = v;
  _blocksize = std::max(_volpkg->getSliceWidth(), _volpkg->getSliceHeight());
  _field = new cv::Vec3f**[_volpkg->getNumberOfSlices()];

  for (int i = 0; i < _volpkg->getNumberOfSlices(); ++i) {
    _field[i] = NULL;
  }
}

Field::~Field() {
  for (int i = 0; i < _volpkg->getNumberOfSlices(); ++i) {
    if (_field[i] != NULL) {
      for (int x = 0; x < _blocksize; ++x) {
        delete[] _field[i][x];
      }
      delete[] _field[i];
    }
  }
  delete[] _field;
}

void Field::clean() {
  int top = *_indexes_used_since_last_clean.begin();

  for (int index = top-1 ; _field[index]; --index) {
    for (int x = 0; x < _blocksize; ++x) {
      delete[] _field[index][x];
    }
    delete[] _field[index];
    _field[index] = NULL;
  }

  _indexes_used_since_last_clean.clear();
}

// interpolation formula from
// http://paulbourke.net/miscellaneous/interpolation/
cv::Vec3f Field::interpolate_at(cv::Vec3f point, int gravity_scale) {
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

  this->loadSlice(x_min);
  this->loadSlice(x_max);

  cv::Vec3f vector =
    _field[x_min][y_min][z_min] * (1 - dx) * (1 - dy) * (1 - dz) +
    _field[x_max][y_min][z_min] * dx       * (1 - dy) * (1 - dz) +
    _field[x_min][y_max][z_min] * (1 - dx) * dy       * (1 - dz) +
    _field[x_min][y_min][z_max] * (1 - dx) * (1 - dy) * dz +
    _field[x_max][y_min][z_max] * dx       * (1 - dy) * dz +
    _field[x_min][y_max][z_max] * (1 - dx) * dy       * dz +
    _field[x_max][y_max][z_min] * dx       * dy       * (1 - dz) +
    _field[x_max][y_max][z_max] * dx       * dy       * dz;

  cv::Vec3f gravity = cv::Vec3f(1,0,0);
  vector = gravity - (gravity.dot(vector)) / (vector.dot(vector)) * vector;
  cv::normalize(vector);
  vector /= gravity_scale;

  return vector;
}

void Field::loadSlice(int index) {
  if (_field[index] == NULL) {
    _field[index] = new cv::Vec3f*[_blocksize];
    for (int j = 0; j < _blocksize; ++j) {
      _field[index][j] = new cv::Vec3f[_blocksize];
      for (int k = 0; k < _blocksize; ++k) {
        _field[index][j][k] = cv::Vec3f(0,0,0);
      }
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (_volpkg->getNormalAtIndex(index), *cloud) == -1) {
      PCL_ERROR ("couldn't read file\n");
      exit(EXIT_FAILURE);
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator point;
    for (point = cloud->begin(); point != cloud->end(); ++point) {
      int x, y, z;
      x = point->x;
      y = point->y;
      z = point->z;

      _field[x][y][z](0) = point->normal[0];
      _field[x][y][z](1) = point->normal[1];
      _field[x][y][z](2) = point->normal[2];
    }
  }

  _indexes_used_since_last_clean.insert(index);
}
