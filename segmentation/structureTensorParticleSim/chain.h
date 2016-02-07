// Chain object maintains a vector of points and their histories.
// step() updates the postitions of particles based on their normal
// vectors. Neighboring particles are kept in line with a "spring".
#pragma once

#ifndef _CHAIN_
#define _CHAIN_

#define DEFAULT_OFFSET -1

#include <list>
#include <vector>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include "volumepkg.h"
#include "particle.h"

class Chain
{
public:
    Chain(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath,
          VolumePkg& volPkg, const double gravity_scale,
          const int32_t threshold, const int32_t endOffset,
          const double spring_constant_k = -0.5);
    void step(void);
    bool isMoving() const;
    cv::Vec3f springForce(const int32_t index) const;
    cv::Vec3f gravity(const int32_t index) const;
    pcl::PointCloud<pcl::PointXYZRGB> orderedPCD() const;

private:
    VolumePkg& _volpkg;
    // History of the chain at each iteration
    std::list<std::vector<Particle>> _history;
    // Parameters for calculating the spring effects
    double _spring_constant_k;
    double _spring_resting_x;
    // Limits the effect of the normal vector
    double _gravity_scale;  // To-Do: Rename.

    // -- Chain Size Information -- //
    int32_t
        _chain_length;  // Number of particles in the chain & width of output
                        // PCD
    int32_t
        _real_iterations;   // Height of the output PCD To-Do: Do we need this?
    int32_t _start_index;   // Starting slice index
    int32_t _target_index;  // Target slice index
    int32_t _threshold;  // To-Do: What is this for now? We may not need this.
};

#endif
