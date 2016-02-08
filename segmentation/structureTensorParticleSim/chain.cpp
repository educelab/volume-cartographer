#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include "chain.h"

// (doc) Why the parameters that we're giving it?
Chain::Chain(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath,
             VolumePkg& volpkg, const double gravity_scale, const int threshold,
             const int endOffset, const double spring_constant_k)
    : _volpkg(volpkg),
      _spring_constant_k(spring_constant_k),
      _gravity_scale(gravity_scale),
      _threshold(threshold)
{
    // Convert the point cloud segPath into a vector of Particles
    std::vector<Particle> init_chain;

    // NOTE: This algorithm uses slice index position as the primary index for
    // points (e.g. point[z][x][y])
    // However, the rest of volpkg stores point information as point[x][y][z].
    // We must make the swap here and
    // also when we generate our output point cloud in this.orderedPCD()
    for (const auto p : *segPath) {
        init_chain.emplace_back(p.z, p.x, p.y);
    }

    // Calculate the spring resting position
    double total_delta = 0;
    for (int32_t i = 1; i < init_chain.size(); ++i) {
        const cv::Vec3d segment = init_chain[i] - init_chain[i - 1];
        total_delta += std::sqrt(segment.dot(segment));
    }
    _spring_resting_x = total_delta / (init_chain.size() - 1);
    _spring_constant_k = spring_constant_k;

    // Add starting chain to _history and setup other parameters
    _history.push_front(init_chain);
    _chain_length = init_chain.size();
    _gravity_scale = gravity_scale;
    _threshold = threshold;

    // Find the lowest slice index in the starting chain
    _start_index = _history.front()[0](0);
    for (int32_t i = 0; i < _chain_length; ++i) {
        if (_history.front()[i](0) < _start_index) {
            _start_index = _history.front()[i](0);
        }
    }

    // Set the slice index we will end at
    // If user does not define endOffset, target index == last slice with a
    // surface normal file
    // Account for zero-indexing and slices lost in calculating normal vector
    _target_index =
        ((endOffset == DEFAULT_OFFSET) ? (volpkg.getNumberOfSlices() - 3)
                                       : (_start_index + endOffset));
    if (_target_index > volpkg.getNumberOfSlices() - 3) {
        _target_index = volpkg.getNumberOfSlices() - 3;
    }

    // Set _realIterationsCount based on starting index, target index, and how
    // frequently we want to sample the segmentation
    _real_iterations =
        int32_t(std::ceil(((_target_index - _start_index) + 1) / _threshold));

    // Go ahead and stop any particles that are already at the target index
    for (int32_t i = 0; i < _chain_length; ++i) {
        if (_history.front()[i](0) >= _target_index) {
            _history.front()[i].stop();
        }
    }
}

// This function defines how particles are updated
// To-Do: Only iterate over particles once
void Chain::step(void)
{
    // Pull the most recent iteration from _history
    std::vector<Particle> update_chain = _history.front();
    std::vector<cv::Vec3d> force_vector{_chain_length, {0, 0, 0}};

    // calculate forces acting on particles
    for (int32_t i = 0; i < _chain_length; ++i) {
        if (update_chain[i].isStopped()) {
            continue;
        }

        force_vector[i] += springForce(i);
        force_vector[i] += gravity(i);
    }

    // update the chain
    for (int32_t i = 0; i < _chain_length; ++i) {
        update_chain[i] += force_vector[i];
        if (std::floor(update_chain[i](0)) >= _target_index) {
            update_chain[i].stop();
        }
    }

    // Add the modified chain back to _history
    _history.push_front(update_chain);
}

// Returns true if any Particle in the chain is still moving
bool Chain::isMoving() const
{
    bool result = true;
    auto c = _history.front();
    return std::all_of(c.begin(), c.end(),
                       [](const Particle p) { return p.isStopped(); });
}

// Returns vector offset that tries to maintain distance between particles as
// _spring_resting_x
// The spring equation (Hooke's law) is -kx where
// k is the spring constant (stiffness)
// x is displacement from rest (starting distance between points)
//
// There are two if blocks to account for the first and last particles in the
// chain
// only having one neighbor.
cv::Vec3d Chain::springForce(const int index) const
{
    cv::Vec3d f(0, 0, 0);
    // Adjust particle with a neighbor to the right
    if (index != _chain_length - 1) {
        const Particle to_right =
            _history.front()[index] - _history.front()[index + 1];
        double length = std::sqrt(to_right.dot(to_right));
        cv::normalize(to_right, to_right,
                      _spring_constant_k * (length - _spring_resting_x));
        f += to_right;
    }
    // Adjust particle with a neighbor to the left
    if (index != 0) {
        const Particle to_left =
            _history.front()[index] - _history.front()[index - 1];
        const double length = sqrt(to_left.dot(to_left));
        cv::normalize(to_left, to_left,
                      _spring_constant_k * (length - _spring_resting_x));
        f += to_left;
    }
    return f;
}

// Project a vector onto the plane described by the normals
cv::Vec3d Chain::gravity(const int32_t index) const
{
    const cv::Vec3d gravity{1, 0, 0};
    const cv::Point3d p = _history.front()[index];

    // Fix Mike's stupid shit - z,x,y --> x,y,z
    const cv::Point3d fixedInterpolatedPoint = {p.y, p.z, p.x};
    cv::Vec3d offset =
        _volpkg.volume()
            .interpolatedEigenPairsAt(fixedInterpolatedPoint, 3)[0]
            .second;
    // convert x,y,z --> z,x,y
    offset = {offset(2), offset(0), offset(1)};

    offset = gravity - (gravity.dot(offset)) / (offset.dot(offset)) * offset;
    cv::normalize(offset);
    return offset * _gravity_scale;
}

// Convert Chain's _history to an ordered Point Cloud object
pcl::PointCloud<pcl::PointXYZRGB> Chain::orderedPCD() const
{
    // Allocate space for one row of the output cloud
    std::vector<pcl::PointXYZRGB> storage_row;
    storage_row.reserve(_chain_length);
    for (int32_t i = 0; i < _chain_length; ++i) {
        pcl::PointXYZRGB point;
        point.z = -1;  // To-Do: Make this a constant
        storage_row.push_back(point);
    }

    // Allocate space for all rows of the output cloud
    // storage will represent the cloud with 2D indexes
    std::vector<std::vector<pcl::PointXYZRGB>> storage;
    storage.reserve(_real_iterations);
    for (int32_t i = 0; i < _real_iterations; ++i) {
        storage.push_back(storage_row);
    }

    // Give the output points an arbitrary color. *To-Do: This is not used ever.
    const static uint32_t COLOR =
        0x00777777;  // grey in PCL's packed RGB representation

    // Push each point in _history into its ordered position in storage if it
    // passes the distance threshold
    for (const auto v : _history) {
        // Add each Particle in the row into storage at the correct position
        // Note: This is where we convert the internal cloud's coordinate
        // ordering back to volume ordering
        for (int32_t i = 0; i < _chain_length; ++i) {
            int32_t currentCell =
                int32_t(((v[i](0)) - _start_index) / _threshold);
            pcl::PointXYZRGB point;
            // point.x == vol[x][ ][ ] == field[ ][x][ ]
            point.x = v[i](1);
            // point.y == vol[ ][y][ ] == field[ ][ ][y]
            point.y = v[i](2);
            // point.z == vol[ ][ ][z] == field[z][ ][ ]
            point.z = v[i](0);
            point.rgb = *(float*)&COLOR;
            storage[currentCell][i] = point;
        }
    }

    // Move points out of storage into the point cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.height = _real_iterations;
    cloud.width = _chain_length;
    cloud.reserve(cloud.height * cloud.width);
    for (int32_t i = 0; i < cloud.height; ++i) {
        for (int32_t j = 0; j < cloud.width; ++j) {
            cloud.push_back(storage[i][j]);
        }
    }

    return cloud;
}
