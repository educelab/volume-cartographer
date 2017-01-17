#pragma once

#include <iostream>

#include "core/types/VolumePkg.hpp"
#include "segmentation/stps/Chain.hpp"
#include "segmentation/stps/Particle.hpp"

namespace volcart
{
namespace segmentation
{

volcart::OrderedPointSet<cv::Vec3d> StructureTensorParticleSim(
    std::vector<cv::Vec3d> segPath,
    const VolumePkg& volpkg,
    double gravityScale = 0.5,
    int threshold = 1,
    int endOffset = -1);
}
}
