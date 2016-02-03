#pragma once

#ifndef _VOLCART_SEGMENTATION_CHAIN_H_
#define _VOLCART_SEGMENTATION_CHAIN_H_

#include <tuple>
#include <opencv2/opencv.hpp>
#include "fittedcurve.h"
#include "common.h"
#include "volumepkg.h"

// this is similar to the chain class in structureTensor
// it tries to find the next position of the particles
// based on slices orthogonal to the chain

namespace volcart
{
namespace segmentation
{
class Chain
{
private:
    VoxelVec particles_;
    VolumePkg& volpkg_;
    int32_t particleCount_;
    int32_t zIndex_;
    FittedCurve<double> curve_;
    bool shouldDraw_;
    bool dumpImages_;

    constexpr static double kDefaultMaxDrift = 0.0;

    cv::Vec3d calculateNormal(const size_t index) const;

public:
    using Iterator = typename VoxelVec::iterator;
    using ConstIterator = typename VoxelVec::const_iterator;
    using DirPosPair = std::tuple<Direction, Voxel>;
    using DirPosPairVec = std::tuple<std::vector<Direction>, VoxelVec>;

    Chain() = default;

    Chain(VolumePkg& pkg, const VoxelVec& pos, const int32_t zIndex,
          const bool shouldDraw = false, const bool dumpImages = false)
        : volpkg_(pkg),
          particleCount_(pos.size()),
          zIndex_(zIndex),
          curve_(pos),
          shouldDraw_(shouldDraw),
          dumpImages_(dumpImages)
    {
        particles_.reserve(particleCount_);
        for (const auto p : curve_.resampledPoints()) {
            particles_.emplace_back(p(0), p(1), zIndex);
        }
    }

    int32_t size(void) const { return particleCount_; }
    Voxel at(const int32_t idx) const
    {
        assert(idx >= 0 && idx < particleCount_ && "index out of range");
        return particles_[idx];
    }

    const VoxelVec& positions() const { return particles_; }
    void setZIndex(int32_t zIndex) { zIndex_ = zIndex; }
    // Iterator functions that reach through to the underlying vector so we can
    // use range-based for with Chain
    ConstIterator begin() const { return particles_.begin(); }
    ConstIterator end() const { return particles_.end(); }
    Iterator begin() { return particles_.begin(); }
    Iterator end() { return particles_.end(); }
    const decltype(curve_)& curve() const { return curve_; }
    std::vector<std::deque<Voxel>> stepAll(const int32_t stepNumLayers,
                                           const int32_t keepNumMaxima) const;

    std::deque<Voxel> step(const int32_t particleIndex,
                           const int32_t stepNumLayers,
                           const int32_t keepNumMaxima) const;

    void setNewPositions(const VoxelVec& newPositions);

    cv::Mat draw(const bool showSpline = false) const;
};
}
}

#endif
