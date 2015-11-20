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
    using TIterator = std::vector<Particle>::iterator;
    using TConstIterator = std::vector<Particle>::const_iterator;
    using DirPosPair = std::tuple<Direction, cv::Vec3d>;
    using DirPosPairVec = std::tuple<std::vector<Direction>, std::vector<cv::Vec3d>>;

    Chain();

    Chain(VolumePkg& pkg, int32_t zIndex);

    int32_t size(void) const { return particleCount_; }

    Particle at(const int32_t idx) const { return particles_.at(idx); }

    void setZIndex(int32_t zIndex) { zIndex_ = zIndex; }

    // Iterator functions that reach through to the underlying vector so we can
    // use range-based for with Chain
    TConstIterator begin() const { return particles_.begin(); }

    TConstIterator end() const { return particles_.end(); }

    TIterator begin() { return particles_.begin(); }

    TIterator end() { return particles_.end(); }

    DirPosPairVec stepAll(const int32_t stepNumLayers) const;

    DirPosPair step(const int32_t particleIndex,
                    const int32_t stepNumLayers,
                    const Direction d=Direction::kDefault,
                    const double maxDrift=kDefaultMaxDrift) const;

    void setNewPositions(std::vector<cv::Vec3d> newPositions);

    void draw() const;

private:
    std::vector<Particle> particles_;
    VolumePkg& volpkg_;
    size_t particleCount_;
    int32_t zIndex_;
    // XXX 4th degree interpolation is about as large as we can go currently.
    // Higher than that and the voxel positions get to be too large when
    // exponentiated.
    FittedCurve<double, 4> curve_;

    constexpr static double kDefaultMaxDrift = 0.0;

    const cv::Vec3d calculateNormal(const size_t index) const;

};

}

}

#endif
