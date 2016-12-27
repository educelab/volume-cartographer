#include "segmentation/stps/StructureTensorParticleSim.h"

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
volcart::OrderedPointSet<volcart::Point3d> structureTensorParticleSim(
    std::vector<volcart::Point3d> segPath,
    const VolumePkg& volpkg,
    double gravity_scale,
    int threshold,
    int endOffset)
{
    // Organize the points into a connected chain
    Chain particle_chain(segPath, volpkg, gravity_scale, threshold, endOffset);

    // The only stop condition is that each particle has reached the target
    // index.
    // Otherwise the loop will stop after an arbitrary number of iterations.
    for (int iteration = 0; particle_chain.isMoving() && iteration < 25000;
         ++iteration) {
        // Make the chain take one iteration forward
        particle_chain.step();
    }

    return particle_chain.orderedPCD();
}
}  // namespace segmentation
}  // namespace volcart
