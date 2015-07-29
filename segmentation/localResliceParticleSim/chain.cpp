#include "chain.h"


DEMO::Chain::Chain(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath, VolumePkg* volpkg, int threshold, int endOffset, int steps_before_reslice) {
  // Convert the point cloud segPath into a vector of Particles
  std::vector<Particle> init_chain;

  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator path_it = segPath->begin(); path_it != segPath->end(); ++path_it){
    init_chain.push_back(cv::Vec3f(path_it->x, path_it->y, path_it->z));
  }

  // used for getting slices for debug output
  _volpkg = volpkg;

  // keeps track of how many iterations have been performed
  _update_count = 0;

  // Add starting chain to _history and setup other parameters
  _history.push_front(init_chain);
  _chain_length      = init_chain.size();
  _threshold         = threshold;

  // normals are updated every iteration by default
  _steps_before_reslice = steps_before_reslice;
  _saved_normals = std::vector<cv::Vec3f>(_chain_length, cv::Vec3f(0,0,0));

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
    _saved_normals[i] = tangent.cross(VC_DIRECTION_K);          \
  } while (0)

// This function defines how particles are updated
void DEMO::Chain::step(DEMO::Field& field) {
  // Pull the most recent iteration from _history
  std::vector<Particle> update_chain = _history.front();
  std::vector<cv::Vec3f> force_vector(_chain_length, cv::Vec3f(0,0,0));

  for(int i = 0; i < _chain_length; ++i) {
    if (update_chain[i].isStopped())
      continue;

    // update normals every _steps_before_reslice steps
    if (_update_count % _steps_before_reslice == 0) {
      // pretend that the normals at the end of the chain
      // are the same as the ones adjacent
      if (i == 0) {
        NORMAL(1);
      } else if (i == _chain_length - 1) {
        NORMAL(_chain_length - 2);
      } else {
        NORMAL(i);
      }
    }

    // reslice and find next position
    Slice s = field.reslice(update_chain[i].position(), _saved_normals[i], VC_DIRECTION_K);

    if (i == 32) {
      s.debugDraw(DEBUG_DRAW_CENTER);
      s.debugAnalysis();
    }

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
  _update_count++;
  _history.push_front(update_chain);
}

// Returns true if any Particle in the chain is still moving
bool DEMO::Chain::isMoving() {
  bool result = true;
  for (int i = 0; i < _chain_length; ++i)
    result &= _history.front()[i].isStopped();
  return !result;
}

// draw a debug window with an option to write to disk
void DEMO::Chain::debug(bool saveOutput) {
  std::vector<Particle> recent = _history.front();
  int z_index = recent[0](VC_INDEX_Z);

  cv::Mat debug = _volpkg->getSliceData(z_index);
  debug *= 1./255;
  debug.convertTo(debug, CV_8UC3);
  cvtColor(debug, debug, CV_GRAY2BGR);

  // draw circles on the debug window for each point
  for (int i = 0; i < recent.size(); ++i) {
    cv::Point position(recent[i](VC_INDEX_X), recent[i](VC_INDEX_Y));
    if (i == 32)
      circle(debug, position, 2, cv::Scalar(0,255,255), -1);
    else
      circle(debug, position, 2, cv::Scalar(0,255,0), -1);
  }

  namedWindow("DEBUG CHAIN", cv::WINDOW_AUTOSIZE);
  imshow("DEBUG CHAIN", debug);

  // option to save output to disk
  if (saveOutput) {
    std::stringstream ss;
    ss << "debug_chain_" << std::setw(3) << std::setfill('0') << _update_count << ".tif";
    cv::imwrite(ss.str(), debug);
  }

  cv::waitKey(0);
}

// Convert Chain's _history to an ordered Point Cloud object
pcl::PointCloud<pcl::PointXYZRGB> DEMO::Chain::orderedPCD() {
  // Allocate space for one row of the output cloud
  std::vector<pcl::PointXYZRGB> storage_row;
  for (int i = 0; i < _chain_length; ++i) {
    pcl::PointXYZRGB point;
    point.z = -1; // To-Do: Make this a constant
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
    // Note: This is where we convert the internal cloud's coordinate ordering back to volume ordering
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
