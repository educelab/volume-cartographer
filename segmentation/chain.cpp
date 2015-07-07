#include "chain.h"

// (doc) Why the parameters that we're giving it?
Chain::Chain(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath, VolumePkg* volpkg, int threshold, int endOffset) {
  // Convert the point cloud segPath into a vector of Particles
  std::vector<Particle> init_chain;
  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator path_it = segPath->begin(); path_it != segPath->end(); ++path_it){
    init_chain.push_back(cv::Vec3f(path_it->x, path_it->y, path_it->z));
  }

  _volpkg = volpkg;

  // Add starting chain to _history and setup other parameters
  _history.push_front(init_chain);
  _chain_length      = init_chain.size();
  _threshold         = threshold;

  // Find the lowest slice index in the starting chain
  _start_index = _history.front()[0](VC_INDEX_Z);
  for (int i = 0; i < _chain_length; ++i)
    if (_history.front()[i](VC_INDEX_Z) < _start_index)
      _start_index = _history.front()[i](VC_INDEX_Z);

  // Set the slice index we will end at
  // If user does not define endOffset, target index == last slice with a surface normal file
  _target_index = ((endOffset == DEFAULT_OFFSET)
                   ? (volpkg->getNumberOfSlices() - 3) // Account for zero-indexing and slices lost in calculating normal vector
                   : (_start_index + endOffset));

  // Set _real_iterations based on starting index, target index, and how frequently we want to sample the segmentation
  _real_iterations = (int)(ceil(((_target_index - _start_index) + 1) / _threshold));
}

#define NORMAL(i) do {                                          \
    cv::Vec3f tangent = update_chain[i+1]-update_chain[i-1];    \
    normal = tangent.cross(VC_DIRECTION_K);                     \
  } while (0)

// This function defines how particles are updated
void Chain::step(Field& field) {
  // Pull the most recent iteration from _history
  std::vector<Particle> update_chain = _history.front();
  std::vector<cv::Vec3f> force_vector(_chain_length, cv::Vec3f(0,0,0));

  // this->debug();

  for(int i = 0; i < _chain_length; ++i) {
    if (update_chain[i].isStopped())
      continue;

    cv::Vec3f normal;
    if (i == 0) {
      NORMAL(1);
    } else if (i == _chain_length - 1) {
      NORMAL(_chain_length - 2);
    } else {
      NORMAL(i);
    }

    Slice s = field.reslice(update_chain[i].position(), normal, VC_DIRECTION_K);
    // s.debugDraw(DEBUG_DRAW_XYZ | DEBUG_DRAW_CENTER);
    // s.debugAnalysis();
    // cv::waitKey(0);

    force_vector[i] += (s.findNextPosition() - update_chain[i].position());
  }

  // update the chain
  for (int i = 0; i < _chain_length; ++i) {
    update_chain[i] += force_vector[i];
    if (floor(update_chain[i](VC_INDEX_Z)) >= _target_index) {
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

void Chain::debug() {
  std::vector<Particle> recent = _history.front();
  int z_index = recent[0](VC_INDEX_Z);

  cv::Mat debug = _volpkg->getSliceData(z_index);
  debug *= 1./255;
  debug.convertTo(debug, CV_8UC3);
  cvtColor(debug, debug, CV_GRAY2BGR);

  for (int i = 0; i < recent.size(); ++i) {
    cv::Point position(recent[i](VC_INDEX_X), recent[i](VC_INDEX_Y));
    if ((int)recent[i](VC_INDEX_Z) == z_index)
      circle(debug, position, 2, cv::Scalar(0,255,0), -1);
    else
      circle(debug, position, 2, cv::Scalar(0,255,255), -1);
  }

  namedWindow("DEBUG CHAIN", cv::WINDOW_AUTOSIZE);
  imshow("DEBUG CHAIN", debug);
  cv::waitKey(0);
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
      int currentCell = (int)(((row_at[i](VC_INDEX_Z)) - _start_index/_threshold)); // *To-Do: Something seems wrong here.
      pcl::PointXYZRGB point;
      point.x = row_at[i](VC_INDEX_X);
      point.y = row_at[i](VC_INDEX_Y);
      point.z = row_at[i](VC_INDEX_Z);
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
