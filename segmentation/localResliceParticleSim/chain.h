#pragma once

#ifndef _VOLCART_SEGMENTATION_CHAIN_H_
#define _VOLCART_SEGMENTATION_CHAIN_H_

#include <tuple>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "field.h"
#include "particle.h"
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

    Chain(VolumePkg& pkg);

    const int32_t size(void) const { return particleCount_; }

    Particle at(const uint32_t idx) const;

    const int32_t zIndex(void) const;

    // Iterator functions that reach through to the underlying vector so we can use range-based for with Chain
    TConstIterator begin() const { return particles_.begin(); }

    TConstIterator end() const { return particles_.end(); }

    TIterator begin() { return particles_.begin(); }

    TIterator end() { return particles_.end(); }

    std::pair<std::vector<Direction>, std::vector<cv::Vec3f>> stepAll() const;

    std::pair<Direction, cv::Vec3f> step(const int32_t index, const Direction d=Direction::kNone,
                                         const double maxDrift=kDefaultMaxDrift) const;

    void setNewPositions(std::vector<cv::Vec3f> newPositions);

private:
    std::vector<Particle> particles_;
    int32_t particleCount_;
    VolumePkg& volpkg_;

    constexpr auto kDefaultMaxDrift = 0.0;

    cv::Vec3f calculateNormal(const uint32_t index) const;

};

}

}

#endif
