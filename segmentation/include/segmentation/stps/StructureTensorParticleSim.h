#pragma once

#include <iostream>

#include "core/types/VolumePkg.h"
#include "segmentation/stps/Chain.h"
#include "segmentation/stps/Particle.h"

namespace volcart
{
namespace segmentation
{
/** @fn volcart::OrderedPointSet<volcart::Point3d> structureTensorParticleSim(
 * std::vector<volcart::Point3d> segPath, const VolumePkg& volpkg,
 * double gravity_scale = 0.5, int threshold = 1, int endOffset=-1)
 *
 * @brief Performs segmentation using the Structure Tensor Particle Simulation
 * Algorithm
 *
 * structureTensorParticleSim uses "particles" to trace out the surfaces in a
 * volume. A Particle Chain maintains their ordering and is responsible for
 * updating theirpositions. This update moves particles according to the
 * estimated normal vector and according to a "spring force" that helps
 * to keep them ordered. The spring force also encourages stagnated
 * particles to move through the volume.
 *
 * @param segPath List of points to be segmented
 * @param volpkg Where the segmentation is stored
 * @param gravity_scale Limits the effects of the normals
 * @param threshold How frequently to sample the segmentation
 *                  Default: 1 to sample the segmentation once per loop
 * @param endOffset Last slice to segment
 *                  Default: -1 to sample all of the slices
 *
 * @ingroup stps
 *
 */
volcart::OrderedPointSet<cv::Vec3d> structureTensorParticleSim(
    std::vector<cv::Vec3d> segPath,
    const VolumePkg& volpkg,
    double gravity_scale = 0.5,
    int threshold = 1,
    int endOffset = -1);
}  // namespace segmentation
}  // namespace volcart
