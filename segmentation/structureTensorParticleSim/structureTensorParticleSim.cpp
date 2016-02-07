#include "structureTensorParticleSim.h"
#include "chain.h"

// structureTensorParticleSim uses "particles" to trace out the surfaces in a
// volume. A Particle Chain maintains their ordering and is responsible for
// updating their positions. This update moves particles according to the
// estimated normal vector and according to a "spring force" that helps to keep
// them ordered. The spring force also encourages stagnated particles to move
// through the volume.
//
// Note: Internally this algorithm tracks a point's position by slice index
// first: p[z][x][y]
// This means that internally, all pcl Points are using the following mapping:
//      p.x == slice index/z pos
//      p.y == slice x pos
//      p.z == slice y pos
// VolPkg clouds, however, use p.x, p.y, and p.z as their expected coordinates
// in slice space, x, y, and z respectively.
// To account for this, the Chain class makes the coordinate swap to ZXY
// ordering in its constructor
// and back to XYZ ordering in Chain.orderedPCD().

namespace volcart
{
namespace segmentation
{
pcl::PointCloud<pcl::PointXYZRGB> structureTensorParticleSim(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath, VolumePkg volpkg,
    const double gravity_scale, const int threshold, const int endOffset)
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
