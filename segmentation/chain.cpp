#include "chain.h"

// (doc) Why the parameters that we're giving it?
Chain::Chain(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath, VolumePkg* volpkg, int gravity_scale, int threshold, int endOffset, double spring_constant_k) {
  // Convert the point cloud segPath into a vector of Particles
  std::vector<Particle> init_chain;
  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator path_it = segPath->begin(); path_it != segPath->end(); ++path_it){
    init_chain.push_back(cv::Vec3f(path_it->x, path_it->y, path_it->z));
  }

  // Calculate the spring resting position
  double total_delta = 0;
  for (int i = 1; i < init_chain.size(); ++i) {
    cv::Vec3f segment = init_chain[i] - init_chain[i-1];
    total_delta += sqrt(segment.dot(segment));
  }
  _spring_resting_x  = total_delta / (init_chain.size() - 1);
  _spring_constant_k = spring_constant_k;
  
  // Add starting chain to _history and setup other parameters
  _history.push_front(init_chain);
  _chain_length      = init_chain.size();
  _gravity_scale     = gravity_scale;
  _threshold         = threshold;

  // Find the lowest slice index in the starting chain 
  _start_index = _history.front()[0](0);
  for (int i = 0; i < _chain_length; ++i)
    if (_history.front()[i](0) < _start_index)
      _start_index = _history.front()[i](0);

  // Set the slice index we will end at
  // If user does not define endOffset, target index == last slice with a surface normal file
  _target_index = ((endOffset == DEFAULT_OFFSET)
                   ? (volpkg->getNumberOfSlices() - 3) // Account for zero-indexing and slices lost in calculating normal vector
                   : (_start_index + endOffset));
  
  // Set _real_iterations based on starting index, target index, and how frequently we want to sample the segmentation
  _real_iterations = (int)(ceil(((_target_index - _start_index) + 1) / _threshold));
}


// This function defines how particles are updated
// To-Do: Only iterate over particles once
void Chain::step(Field& field) {
  // Pull the most recent iteration from _history
  std::vector<Particle> update_chain = _history.front();

  // Apply the springForce to each Particle
  for(int i = 0; i < _chain_length; ++i) {
    if (update_chain[i].isStopped()) continue;
    update_chain[i] += this->springForce(i);
  }

  // Move each particle along plane defined by estimated surface normal
  cv::Vec3f gravity = cv::Vec3f(1,0,0); // To-Do: Rename gravity?
  for(int i = 0; i < _chain_length; ++i) {
    if (update_chain[i].isStopped())
      continue;

    // Project known vector onto estimated plane
    cv::Vec3f offset = field.interpolate_at(update_chain[i].position());
    offset = gravity - (gravity.dot(offset)) / (offset.dot(offset)) * offset;
    cv::normalize(offset);
    offset /= _gravity_scale;
    update_chain[i] += offset;
  }

  // Apply the springForce to each Particle again because why?
  for(int i = 0; i < _chain_length; ++i) {
    if (update_chain[i].isStopped()) continue;
    update_chain[i] += this->springForce(i);
  }

  // Stop each particle if it's hit the target slice index
  for (int i = 0; i < _chain_length; ++i) {
    if (floor(update_chain[i](0)) >= _target_index) {
      update_chain[i].stop();
    }
  }

  // Add the modified chain back to _history
  _history.push_front(update_chain);
}

// Returns true if any Particle in the chain is still moving
bool Chain::isMoving() {
  bool result = true;
  for (int i = 0; i < _chain_length; ++i)
    result &= _history.front()[i].isStopped();
  return !result;
}

// Returns vector offset that tries to maintain distance between particles as _spring_resting_x
// The spring equation (Hooke's law) is -kx where
// k is the spring constant (stiffness)
// x is displacement from rest (starting distance between points)
//
// There are two if blocks to account for the first and last particles in the chain
// only having one neighbor.
cv::Vec3f Chain::springForce(int index) {
  cv::Vec3f f(0,0,0);
  // Adjust particle with a neighbor to the right
  if (index != _chain_length - 1) {
    cv::Vec3f to_right = _history.front()[index] - _history.front()[index+1];
    double length = sqrt(to_right.dot(to_right));
    normalize(to_right, to_right, _spring_constant_k * (length - _spring_resting_x));
    f += to_right;
  }
  // Adjust particle with a neighbor to the left
  if (index != 0) {
    cv::Vec3f to_left = _history.front()[index] - _history.front()[index - 1];
    double length = sqrt(to_left.dot(to_left));
    normalize(to_left, to_left, _spring_constant_k * (length - _spring_resting_x));
    f += to_left;
  }
  return f;
}

// Convert Chain's _history to an ordered Point Cloud object
pcl::PointCloud<pcl::PointXYZRGB> Chain::orderedPCD() {
  // Allocate space for one row of the output cloud
  std::vector<pcl::PointXYZRGB> storage_row;
  for (int i = 0; i < _chain_length; ++i) {
    pcl::PointXYZRGB point;
    point.x = -1; // To-Do: Make this a constant
    storage_row.push_back(point);
  }

  // Allocate space for all rows of the output cloud
  // storage will represent the cloud with 2D indexes
  std::vector<std::vector<pcl::PointXYZRGB> > storage;
  for (int i = 0; i < _real_iterations; ++i) {
    storage.push_back(storage_row);
  }

  // Give the output points an abitrary color. *To-Do: This is not used ever.
  uint32_t COLOR = 0x00777777; // grey in PCL's packed RGB representation

  // Push each point in _history into its ordered position in storage if it passes the distance threshold
  for (std::list<std::vector<Particle> >::iterator it = _history.begin(); it != _history.end(); ++it) {
    // Get a row of Particles in _history
    std::vector<Particle> row_at = *it;

    // Add each Particle in the row into storage at the correct position
    for (int i = 0; i < _chain_length; ++i) {
      int currentCell = (int)(((row_at[i](0)) - _start_index/_threshold)); // *To-Do: Something seems wrong here.
      pcl::PointXYZRGB point;
      point.x = row_at[i](0);
      point.y = row_at[i](1);
      point.z = row_at[i](2);
      point.rgb = *(float*)&COLOR;
      storage[currentCell][i] = point;
    }
  }

  // Move points out of storage into the point cloud
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
