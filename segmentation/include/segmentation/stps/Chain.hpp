// Chain object maintains a vector of points and their histories.
// step() updates the postitions of particles based on their normal
// vectors. Neighboring particles are kept in line with a "spring".
#pragma once

#define DEFAULT_OFFSET (-1)

#include <list>
#include <vector>

#include <opencv2/core.hpp>

#include "core/types/VolumePkg.hpp"
#include "segmentation/stps/Particle.hpp"

class Chain
{
public:
    Chain(
        std::vector<cv::Vec3d> segPath,
        const VolumePkg& volpkg,
        double gravityScale,
        int threshold,
        int endOffset,
        double springConstantK = -0.5);
    void step();
    bool isMoving();
    cv::Vec3d springForce(size_t index);
    cv::Vec3d gravity(size_t index);
    volcart::OrderedPointSet<cv::Vec3d> orderedPCD();

private:
    const VolumePkg& volpkg_;

    // History of the chain at each iteration
    std::list<std::vector<Particle>> history_;
    // Parameters for calculating the spring effects
    double springConstantK_;
    double springRestingX_;
    // Limits the effect of the normal vector
    double gravityScale_;  // To-Do: Rename.

    // -- Chain Size Information -- //
    size_t chainLength_;  // Number of particles in the chain & width of output
                          // PCD
    // Height of the output PCD To-Do: Do we need this?
    size_t realIterations_;
    size_t startIndex_;   // Starting slice index
    size_t targetIndex_;  // Target slice index
    int threshold_;       // To-Do: What is this for now? We may not need this.
};
