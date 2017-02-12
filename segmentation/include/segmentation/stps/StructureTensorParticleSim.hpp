#pragma once

#include <iostream>

#include "core/types/VolumePkg.hpp"
#include "segmentation/stps/Chain.hpp"
#include "segmentation/stps/Particle.hpp"

namespace volcart
{
namespace segmentation
{
/**
 * @brief Structure Tensor %Particle Simulation (STPS) segmentation
 *
 * This algorithm propagates a chain of particles forward through a volume from
 * their starting position to some offset position in the positive Z-axis
 * direction. The particles use a local 3D structure tensor to determine their
 * next propagated position and are kept in close proximity using Hooke's law.
 * For more information on how particles are propagated, see Chain().
 *
 * @param segPath Input seed points
 * @param volpkg VolumePkg from which to pull intensity information
 * @param gravityScale Gravity scale factor
 * @param threshold Distance threshold at which to sample the chain (from
 * previous sampled position)
 * @param endOffset Final offset from starting z-index. Default propagates to
 * last possible z-index.
 *
 * @ingroup stps
 */
volcart::OrderedPointSet<cv::Vec3d> StructureTensorParticleSim(
    std::vector<cv::Vec3d> segPath,
    const VolumePkg& volpkg,
    double gravityScale = 0.5,
    int threshold = 1,
    int endOffset = -1);
}
}
