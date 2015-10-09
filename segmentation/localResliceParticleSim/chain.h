#pragma once

#ifndef _VOLCART_SEGMENTATION_CHAIN_H_
#define _VOLCART_SEGMENTATION_CHAIN_H_

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "field.h"
#include "particle.h"

// this is similar to the chain class in structureTensor
// it tries to find the next position of the particles
// based on slices orthogonal to the chain

namespace volcart {

namespace segmentation {

class Chain {
public:
    Chain(VolumePkg& pkg);

    int32_t size(void) const { return particleCount_; }

    Particle at(uint32_t idx) const;

    int32_t zIndex(void) const;

private:
    std::vector<Particle> particles_;
    int32_t particleCount_;
    VolumePkg& volpkg_;

    cv::Vec3f calculateNormal(uint64_t, std::vector<Particle>);

};

}

}

#endif
