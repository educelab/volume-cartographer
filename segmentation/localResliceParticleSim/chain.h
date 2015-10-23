#pragma once

#ifndef _VOLCART_SEGMENTATION_CHAIN_H_
#define _VOLCART_SEGMENTATION_CHAIN_H_

#include <tuple>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "particle.h"
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
    using DirPosPair = std::tuple<Direction, cv::Vec3f>;
    using DirPosVecPair = std::tuple<std::vector<Direction>, std::vector<cv::Vec3f>>;

    Chain();

    Chain(VolumePkg& pkg);

    int32_t size(void) const { return particleCount_; }

    Particle at(const int32_t idx) const;

    int32_t zIndex(void) const;

    // Iterator functions that reach through to the underlying vector so we can
    // use range-based for with Chain
    TConstIterator begin() const { return particles_.begin(); }

    TConstIterator end() const { return particles_.end(); }

    TIterator begin() { return particles_.begin(); }

    TIterator end() { return particles_.end(); }

    DirPosVecPair stepAll() const;

    DirPosPair step(const int32_t particleIndex,
                    const Direction d=Direction::kNone,
                    const double maxDrift=kDefaultMaxDrift) const;

    void setNewPositions(std::vector<cv::Vec3f> newPositions);

    void draw() const;

private:
    std::vector<Particle> particles_;
    VolumePkg& volpkg_;
    int32_t particleCount_;

    constexpr static double kDefaultMaxDrift = 0.0;

    cv::Vec3f calculateNormal(const int32_t index) const;

};

}

}

#endif
