/** @file StructureTensorParticleSim.cpp */
#include "segmentation/stps/StructureTensorParticleSim.hpp"

namespace volcart
{
namespace segmentation
{
volcart::OrderedPointSet<cv::Vec3d> StructureTensorParticleSim(
    std::vector<cv::Vec3d> segPath,
    const VolumePkg& volpkg,
    double gravityScale,
    int threshold,
    int endOffset)
{
    // Organize the points into a connected chain
    Chain particleChain(
        std::move(segPath), volpkg, gravityScale, threshold, endOffset);

    // The only stop condition is that each particle has reached the target
    // index.
    // Otherwise the loop will stop after an arbitrary number of iterations.
    for (int iteration = 0; particleChain.isMoving() && iteration < 25000;
         ++iteration) {
        // Make the chain take one iteration forward
        particleChain.step();
    }

    return particleChain.orderedPCD();
}
}  // namespace segmentation
}  // namespace volcart
