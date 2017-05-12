#include "vc/segmentation/stps/StructureTensorParticleSim.hpp"

namespace vcs = volcart::segmentation;

volcart::OrderedPointSet<cv::Vec3d> vcs::StructureTensorParticleSim(
    std::vector<cv::Vec3d> segPath,
    const volcart::VolumePkg& volpkg,
    double gravityScale,
    int threshold,
    int endOffset)
{
    // Organize the points into a connected chain
    vcs::Chain particleChain(
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
