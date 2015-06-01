#include "chain.h"

Chain::Chain(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath, double spring_constant_k) {
  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator path_it = segPath->begin(); path_it != segPath->end(); ++path_it){
    _chain.push_back(cv::Vec3f(path_it->x, path_it->y, path_it->z));
  }
  
  double total_delta = 0;
  for (int i = 1; i < _chain.size(); ++i) {
    cv::Vec3f segment = _chain[i] - _chain[i-1];
    total_delta += sqrt( segment.dot( segment ) );
  }
  _spring_resting_x = total_delta / (_chain.size() - 1);
  _spring_constant_k = spring_constant_k;
}

void Chain::update(Field& field, int gravity_scale) {
  for(int i = 0; i < _chain.size(); ++i) {
    if (_chain[i].status()) continue;
    _chain[i] += this->springForce(i);
  }

  for(int i = 0; i < _chain.size(); ++i) {
    if (_chain[i].status()) continue;
    _chain[i] += field.interpolate_at(_chain[i].position(), gravity_scale);
  }

  for(int i = 0; i < _chain.size(); ++i) {
    if (_chain[i].status()) continue;
    _chain[i] += this->springForce(i);
  }
}

bool Chain::isMoving() {
  bool result = true;
  for (int i = 0; i < _chain.size(); ++i)
    result &= _chain[i].status();
  return !result;  
}

int Chain::size() {
  return _chain.size();
}

int Chain::minIndex() {
  int mi = _chain[0](0);
  for (int i = 0; i < _chain.size(); ++i) {
    if (_chain[i](0) > mi) {
      mi = _chain[i](0);
    }
  }
  return mi;
}

Particle Chain::operator[](int index) {
  return _chain[index];
}

cv::Vec3f Chain::springForce(int index) {
  cv::Vec3f f(0,0,0);
  if (index != _chain.size() - 1) {
    cv::Vec3f to_right = _chain[index] - _chain[index+1];
    double length = sqrt(to_right.dot(to_right));
    normalize(to_right, to_right, _spring_constant_k * (length - _spring_resting_x));
    f += to_right;
  }
  if (index != 0) {
    cv::Vec3f to_left = _chain[index] - _chain[index - 1];
    double length = sqrt(to_left.dot(to_left));
    normalize(to_left, to_left, _spring_constant_k * (length - _spring_resting_x));
    f += to_left;
  }
  return f;
}
