#pragma once

#ifndef _VOLCART_SEGMENTATION_CHAIN_H_
#define _VOLCART_SEGMENTATION_CHAIN_H_

#include <tuple>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "particle.h"
#include "fittedcurve.h"
#include "volumepkg.h"
#include "common.h"


// this is similar to the chain class in structureTensor
// it tries to find the next position of the particles
// based on slices orthogonal to the chain

namespace volcart {

namespace segmentation {

class Chain {
public:
    using Iterator       = typename VoxelVec::iterator;
    using ConstIterator  = typename VoxelVec::const_iterator;
    using DirPosPair     = std::tuple<Direction, Voxel>;
    using DirPosPairVec  = std::tuple<std::vector<Direction>, VoxelVec>;

    Chain();

    Chain(VolumePkg& pkg, int32_t zIndex);

    Chain(VolumePkg& pkg, const VoxelVec& pos, int32_t zIndex);

    int32_t size(void) const { return particleCount_; }

	Voxel at(const int32_t idx) const { return particles_[idx]; }

    const VoxelVec& positions() const { return particles_; }

    void setZIndex(int32_t zIndex) { zIndex_ = zIndex; }

    // Iterator functions that reach through to the underlying vector so we can
    // use range-based for with Chain
    ConstIterator begin() const { return particles_.begin(); }

    ConstIterator end() const { return particles_.end(); }

    Iterator begin() { return particles_.begin(); }

    Iterator end() { return particles_.end(); }

    const FittedCurve<>& fittedCurve() const { return curve_; }

    std::vector<VoxelVec> stepAll(const int32_t stepNumLayers,
                                  const int32_t keepNumMaxima) const;

    VoxelVec step(const int32_t particleIndex, const int32_t stepNumLayers,
                  const int32_t keepNumMaxima) const;

    void setNewPositions(const VoxelVec& newPositions);

    void draw() const;

private:
	VoxelVec particles_;
    VolumePkg& volpkg_;
    size_t particleCount_;
    int32_t zIndex_;
    FittedCurve<> curve_;

    constexpr static double kDefaultMaxDrift = 0.0;

    const Voxel calculateNormal(const size_t index) const;

};

}

}

#endif
