#include <cmath>

#include "segmentation/stps/Chain.h"

// (doc) Why the parameters that we're giving it?
Chain::Chain(
    std::vector<cv::Vec3d> segPath,
    const VolumePkg& volpkg,
    double gravityScale,
    int threshold,
    int endOffset,
    double springConstantK)
    : volpkg_{volpkg}
{
    // Convert the point cloud segPath into a vector of Particles
    std::vector<Particle> initChain;
    initChain.reserve(segPath.size());
    for (const auto& p : segPath) {
        initChain.emplace_back(std::move(p));
    }

    // Calculate the spring resting position
    double totalDelta = 0;
    for (size_t i = 1; i < initChain.size(); ++i) {
        cv::Vec3d segment = initChain[i] - initChain[i - 1];
        totalDelta += std::sqrt(segment.dot(segment));
    }
    springRestingX_ = totalDelta / (initChain.size() - 1);
    springConstantK_ = springConstantK;

    // Add starting chain to _history and setup other parameters
    history_.push_front(initChain);
    chainLength_ = initChain.size();
    gravityScale_ = gravityScale;
    threshold_ = threshold;

    // Find the lowest slice index in the starting chain
    startIndex_ = static_cast<int>(history_.front()[0](2));
    for (size_t i = 0; i < chainLength_; ++i) {
        if (history_.front()[i](2) < startIndex_) {
            startIndex_ = static_cast<int>(history_.front()[i](2));
        }
    }

    // Set the slice index we will end at
    // If user does not define endOffset, target index == last slice with a
    // surface normal file
    targetIndex_ =
        ((endOffset == DEFAULT_OFFSET)
             ? (volpkg.getNumberOfSlices() - 1)  // Account for zero-indexing
                                                 // and slices lost in
                                                 // calculating normal vector
             : (startIndex_ + endOffset));
    if (static_cast<int>(targetIndex_) >= volpkg.getNumberOfSlices()) {
        targetIndex_ = volpkg.getNumberOfSlices() - 1;
    }

    // Set _realIterationsCount based on starting index, target index, and how
    // frequently we want to sample the segmentation
    realIterations_ = static_cast<size_t>(
        ceil(((targetIndex_ - startIndex_) + 1) / threshold_));

    // Go ahead and stop any particles that are already at the target index
    for (size_t i = 0; i < chainLength_; ++i) {
        if (history_.front()[i](2) >= targetIndex_) {
            history_.front()[i].stop();
        }
    }
}

// This function defines how particles are updated
// To-Do: Only iterate over particles once
void Chain::step()
{
    // Pull the most recent iteration from _history
    std::vector<Particle> updateChain = history_.front();
    std::vector<cv::Vec3d> forceVector(chainLength_, cv::Vec3d(0, 0, 0));

    // calculate forces acting on particles
    for (size_t i = 0; i < chainLength_; ++i) {
        if (updateChain[i].isStopped()) {
            continue;
        }

        forceVector[i] += this->springForce(i);
        forceVector[i] += this->gravity(i);
    }

    // update the chain
    for (size_t i = 0; i < chainLength_; ++i) {
        updateChain[i] += forceVector[i];
        if (floor(updateChain[i](2)) >= targetIndex_) {
            updateChain[i].stop();
        }
    }

    // Add the modified chain back to _history
    history_.push_front(updateChain);
}

// Returns true if any Particle in the chain is still moving
bool Chain::isMoving()
{
    return std::any_of(
        std::begin(history_.front()), std::end(history_.front()),
        [](auto p) { return !p.isStopped(); });
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
cv::Vec3d Chain::springForce(size_t index)
{
    cv::Vec3d f(0, 0, 0);
    // Adjust particle with a neighbor to the right
    if (index != chainLength_ - 1) {
        auto sindex = static_cast<size_t>(index);
        cv::Vec3d toRight =
            history_.front()[sindex] - history_.front()[sindex + 1];
        double length = sqrt(toRight.dot(toRight));
        normalize(
            toRight, toRight, springConstantK_ * (length - springRestingX_));
        f += toRight;
    }
    // Adjust particle with a neighbor to the left
    if (index != 0) {
        cv::Vec3d toLeft =
            history_.front()[index] - history_.front()[index - 1];
        double length = sqrt(toLeft.dot(toLeft));
        normalize(
            toLeft, toLeft, springConstantK_ * (length - springRestingX_));
        f += toLeft;
    }
    return f;
}

// Project a vector onto the plane described by the structure tensor-computed
// normals
cv::Vec3d Chain::gravity(size_t index)
{
    cv::Vec3d gravity = cv::Vec3d(0, 0, 1);  // To-Do: Rename gravity?

    cv::Vec3d offset =
        volpkg_.volume()
            .interpolatedEigenPairsAt(history_.front()[index].position(), 3)[0]
            .second;

    offset = gravity - (gravity.dot(offset)) / (offset.dot(offset)) * offset;
    cv::normalize(offset);
    return offset * gravityScale_;
}

// Convert Chain's _history to an ordered Point Cloud object
volcart::OrderedPointSet<cv::Vec3d> Chain::orderedPCD()
{
    // Allocate space for one row of the output cloud
    std::vector<cv::Vec3d> storageRow;
    for (size_t i = 0; i < chainLength_; ++i) {
        cv::Vec3d point;
        point[2] = -1;  // To-Do: Make this a constant
        storageRow.push_back(point);
    }

    // Allocate space for all rows of the output cloud
    // storage will represent the cloud with 2D indexes
    std::vector<std::vector<cv::Vec3d>> storage;
    for (size_t i = 0; i < realIterations_; ++i) {
        storage.push_back(storageRow);
    }

    // Push each point in _history into its ordered position in storage if it
    // passes the distance threshold
    for (auto rowAt : history_) {
        // Add each Particle in the row into storage at the correct position
        for (size_t i = 0; i < chainLength_; ++i) {
            int currentCell = static_cast<int>(
                ((rowAt[i](2)) -
                 startIndex_ /
                     threshold_));  // *To-Do: Something seems wrong here.
            storage[currentCell][i] = cv::Vec3d(rowAt[i].position());
        }
    }

    // Move points out of storage into the point cloud
    volcart::OrderedPointSet<cv::Vec3d> cloud;
    for (size_t i = 0; i < realIterations_; ++i) {
        for (size_t j = 0; j < chainLength_; ++j) {
            cloud[j + (i * chainLength_)] = storage[i][j];
        }
    }
    return cloud;
}
