#include "chain.h"

Chain::Chain(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath, VolumePkg* volpkg, int gravity_scale, int threshold, int endOffset, double spring_constant_k) {
  std::vector<Particle> init_chain;
  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator path_it = segPath->begin(); path_it != segPath->end(); ++path_it){
    init_chain.push_back(cv::Vec3f(path_it->x, path_it->y, path_it->z));
  }

  double total_delta = 0;
  for (int i = 1; i < init_chain.size(); ++i) {
    cv::Vec3f segment = init_chain[i] - init_chain[i-1];
    total_delta += sqrt(segment.dot(segment));
  }

  _history.push_front(init_chain);
  _spring_resting_x  = total_delta / (init_chain.size() - 1);
  _spring_constant_k = spring_constant_k;
  _chain_length      = init_chain.size();
  _gravity_scale     = gravity_scale;
  _threshold         = threshold;
  _min_index         = _history.front()[0](0);

  for (int i = 0; i < _chain_length; ++i)
    if (_history.front()[i](0) < _min_index)
      _min_index = _history.front()[i](0);

  _max_index = ((endOffset == -1)
                ? (volpkg->getNumberOfSlices() - 3)
                : (_min_index + endOffset));
  _real_iterations = (int)(ceil(((_max_index - _min_index) + 1) / _threshold));
}



void Chain::update(Field& field) {
  std::vector<Particle> update_chain = _history.front();

  for(int i = 0; i < _chain_length; ++i) {
    if (update_chain[i].status()) continue;
    update_chain[i] += this->springForce(i);
  }

  cv::Vec3f gravity = cv::Vec3f(1,0,0);
  for(int i = 0; i < _chain_length; ++i) {
    if (update_chain[i].status())
      continue;

    cv::Vec3f vector = field.interpolate_at(update_chain[i].position());
    vector = gravity - (gravity.dot(vector)) / (vector.dot(vector)) * vector;
    cv::normalize(vector);
    vector /= _gravity_scale;
    update_chain[i] += vector;
  }

  for(int i = 0; i < _chain_length; ++i) {
    if (update_chain[i].status()) continue;
    update_chain[i] += this->springForce(i);
  }

  for (int i = 0; i < _chain_length; ++i) {
    if (floor(update_chain[i](0)) > _max_index) {
      update_chain[i].invalidate();
    }
  }

  _history.push_front(update_chain);
}

bool Chain::isMoving() {
  bool result = true;
  for (int i = 0; i < _chain_length; ++i)
    result &= _history.front()[i].status();
  return !result;
}

cv::Vec3f Chain::springForce(int index) {
  cv::Vec3f f(0,0,0);
  if (index != _chain_length - 1) {
    cv::Vec3f to_right = _history.front()[index] - _history.front()[index+1];
    double length = sqrt(to_right.dot(to_right));
    normalize(to_right, to_right, _spring_constant_k * (length - _spring_resting_x));
    f += to_right;
  }
  if (index != 0) {
    cv::Vec3f to_left = _history.front()[index] - _history.front()[index - 1];
    double length = sqrt(to_left.dot(to_left));
    normalize(to_left, to_left, _spring_constant_k * (length - _spring_resting_x));
    f += to_left;
  }
  return f;
}

pcl::PointCloud<pcl::PointXYZRGB> Chain::orderedPCD() {
  std::vector<pcl::PointXYZRGB> storage_row;
  for (int i = 0; i < _chain_length; ++i) {
    pcl::PointXYZRGB point;
    point.x = -1;
    storage_row.push_back(point);
  }

  std::vector<std::vector<pcl::PointXYZRGB> > storage;
  for (int i = 0; i < _real_iterations; ++i) {
    storage.push_back(storage_row);
  }

  uint32_t COLOR = 0x00777777;
  for (std::list<std::vector<Particle> >::iterator it = _history.begin(); it != _history.end(); ++it) {
    std::vector<Particle> row_at = *it;

    for (int i = 0; i < _chain_length; ++i) {
      int currentCell = (int)(((row_at[i](0)) - _min_index/_threshold));
      pcl::PointXYZRGB point;
      point.x = row_at[i](0);
      point.y = row_at[i](1);
      point.z = row_at[i](2);
      point.rgb = *(float*)&COLOR;
      storage[currentCell][i] = point;
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.height = _real_iterations;
  cloud.width = _chain_length;
  cloud.points.resize(cloud.height * cloud.width);
  for (int i = 0; i < cloud.height; ++i) {
    for (int j = 0; j < cloud.width; ++j) {
      cloud.points[j+(i*cloud.width)] = storage[i][j];
    }
  }
  return cloud;
}
