#include "segmentation/stps/StructureTensorParticleSim.hpp"

// structureTensorParticleSim uses "particles" to trace out the surfaces in a
// volume.
// A Particle Chain maintains their ordering and is responsible for updating
// their
// positions. This update moves particles according to the estimated normal
// vector
// and according to a "spring force" that helps to keep them ordered. The spring
// force also encourages stagnated particles to move through the volume.

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
