#include "segmentation/stps/chain.h"

// (doc) Why the parameters that we're giving it?
Chain::Chain(
    std::vector<volcart::Point3d> segPath,
    const VolumePkg& volpkg,
    double gravity_scale,
    int threshold,
    int endOffset,
    double spring_constant_k)
    : _volpkg(volpkg)
{
    // Convert the point cloud segPath into a vector of Particles
    std::vector<Particle> init_chain;

    for (auto p : segPath) {
        init_chain.push_back(p.toCvVec());
    }

    // Calculate the spring resting position
    double total_delta = 0;
    for (size_t i = 1; i < init_chain.size(); ++i) {
        cv::Vec3d segment = init_chain[i] - init_chain[i - 1];
        total_delta += sqrt(segment.dot(segment));
    }
    _spring_resting_x = total_delta / (init_chain.size() - 1);
    _spring_constant_k = spring_constant_k;

    // Add starting chain to _history and setup other parameters
    _history.push_front(init_chain);
    _chain_length = init_chain.size();
    _gravity_scale = gravity_scale;
    _threshold = threshold;

    // Find the lowest slice index in the starting chain
    _start_index = _history.front()[0](2);
    for (int i = 0; i < _chain_length; ++i)
        if (_history.front()[i](2) < _start_index)
            _start_index = _history.front()[i](2);

    // Set the slice index we will end at
    // If user does not define endOffset, target index == last slice with a
    // surface normal file
    _target_index =
        ((endOffset == DEFAULT_OFFSET)
             ? (volpkg.getNumberOfSlices() - 1)  // Account for zero-indexing
                                                 // and slices lost in
                                                 // calculating normal vector
             : (_start_index + endOffset));
    if (_target_index >= volpkg.getNumberOfSlices())
        _target_index = volpkg.getNumberOfSlices() - 1;

    // Set _realIterationsCount based on starting index, target index, and how
    // frequently we want to sample the segmentation
    _real_iterations = static_cast<int>(
        ceil(((_target_index - _start_index) + 1) / _threshold));

    // Go ahead and stop any particles that are already at the target index
    for (int i = 0; i < _chain_length; ++i)
        if (_history.front()[i](2) >= _target_index)
            _history.front()[i].stop();
}

// This function defines how particles are updated
// To-Do: Only iterate over particles once
void Chain::step()
{
    // Pull the most recent iteration from _history
    std::vector<Particle> update_chain = _history.front();
    std::vector<cv::Vec3d> force_vector(_chain_length, cv::Vec3d(0, 0, 0));

    // calculate forces acting on particles
    for (int i = 0; i < _chain_length; ++i) {
        if (update_chain[i].isStopped())
            continue;

        force_vector[i] += this->springForce(i);
        force_vector[i] += this->gravity(i);
    }

    // update the chain
    for (int i = 0; i < _chain_length; ++i) {
        update_chain[i] += force_vector[i];
        if (floor(update_chain[i](2)) >= _target_index) {
            update_chain[i].stop();
        }
    }

    // Add the modified chain back to _history
    _history.push_front(update_chain);
}

// Returns true if any Particle in the chain is still moving
bool Chain::isMoving()
{
    bool result = true;
    for (int i = 0; i < _chain_length; ++i)
        result &= _history.front()[i].isStopped();
    return !result;
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
cv::Vec3d Chain::springForce(int index)
{
    cv::Vec3d f(0, 0, 0);
    // Adjust particle with a neighbor to the right
    if (index != _chain_length - 1) {
        cv::Vec3d to_right =
            _history.front()[index] - _history.front()[index + 1];
        double length = sqrt(to_right.dot(to_right));
        normalize(
            to_right, to_right,
            _spring_constant_k * (length - _spring_resting_x));
        f += to_right;
    }
    // Adjust particle with a neighbor to the left
    if (index != 0) {
        cv::Vec3d to_left =
            _history.front()[index] - _history.front()[index - 1];
        double length = sqrt(to_left.dot(to_left));
        normalize(
            to_left, to_left,
            _spring_constant_k * (length - _spring_resting_x));
        f += to_left;
    }
    return f;
}

// Project a vector onto the plane described by the structure tensor-computed
// normals
cv::Vec3d Chain::gravity(int index)
{
    cv::Vec3d gravity = cv::Vec3d(0, 0, 1);  // To-Do: Rename gravity?

    cv::Vec3d offset =
        _volpkg.volume()
            .interpolatedEigenPairsAt(_history.front()[index].position(), 3)[0]
            .second;

    offset = gravity - (gravity.dot(offset)) / (offset.dot(offset)) * offset;
    cv::normalize(offset);
    return offset * _gravity_scale;
}

// Convert Chain's _history to an ordered Point Cloud object
volcart::OrderedPointSet<volcart::Point3d> Chain::orderedPCD()
{
    // Allocate space for one row of the output cloud
    std::vector<volcart::Point3d> storage_row;
    for (int i = 0; i < _chain_length; ++i) {
        volcart::Point3d point;
        point[2] = -1;  // To-Do: Make this a constant
        storage_row.push_back(point);
    }

    // Allocate space for all rows of the output cloud
    // storage will represent the cloud with 2D indexes
    std::vector<std::vector<volcart::Point3d>> storage;
    for (int i = 0; i < _real_iterations; ++i) {
        storage.push_back(storage_row);
    }

    // Push each point in _history into its ordered position in storage if it
    // passes the distance threshold
    for (std::list<std::vector<Particle>>::iterator it = _history.begin();
         it != _history.end(); ++it) {
        // Get a row of Particles in _history
        std::vector<Particle> row_at = *it;

        // Add each Particle in the row into storage at the correct position
        for (int i = 0; i < _chain_length; ++i) {
            int currentCell = static_cast<int>(
                ((row_at[i](2)) -
                 _start_index /
                     _threshold));  // *To-Do: Something seems wrong here.
            storage[currentCell][i] = volcart::Point3d(row_at[i].position());
        }
    }

    // Move points out of storage into the point cloud
    volcart::OrderedPointSet<volcart::Point3d> cloud;
    for (int i = 0; i < _real_iterations; ++i) {
        for (int j = 0; j < _chain_length; ++j) {
            cloud[j + (i * _chain_length)] = storage[i][j];
        }
    }
    return cloud;
}
