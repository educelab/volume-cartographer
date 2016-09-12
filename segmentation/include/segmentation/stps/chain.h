// Chain object maintains a vector of points and their histories.
// step() updates the postitions of particles based on their normal
// vectors. Neighboring particles are kept in line with a "spring".
#pragma once

#define DEFAULT_OFFSET -1

#include <list>
#include <vector>
#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "volumepkg/volumepkg.h"
#include "segmentation/stps/particle.h"

class Chain {
     public:
        Chain(volcart::OrderedPointSet<volcart::Point3d> segPath ,  const VolumePkg& volpkg,  double gravity_scale,  int threshold,  int endOffset,  double spring_constant_k = -0.5);
        void step();
        bool isMoving();
        cv::Vec3d springForce(int);
        cv::Vec3d gravity(int);
        volcart::OrderedPointSet<volcart::Point3d> orderedPCD();

     private:

        const VolumePkg& _volpkg;

        // History of the chain at each iteration
        std::list<std::vector<Particle> > _history;
        // Parameters for calculating the spring effects
        double _spring_constant_k;
        double _spring_resting_x;
        // Limits the effect of the normal vector
        double _gravity_scale; // To-Do: Rename.

         // -- Chain Size Information -- //
        int _chain_length; // Number of particles in the chain & width of output PCD
        int _real_iterations; // Height of the output PCD To-Do: Do we need this?
        int _start_index; // Starting slice index
        int _target_index; // Target slice index
        int _threshold; // To-Do: What is this for now? We may not need this.
};
