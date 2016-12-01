
#pragma once

#define DEFAULT_OFFSET -1

#include <list>
#include <vector>

#include <opencv2/core.hpp>

#include "core/types/VolumePkg.h"
#include "segmentation/stps/Particle.h"
/**
 * @brief Creates a vector to hold points currently being segmented
 *
 * Chain object maintains a vector of points and their histories.
 * step() updates the postitions of particles based on their normal
 * vectors. Neighboring particles are kept in line with a "spring".
 *
 * @ingroup stps
 */
class Chain
{
public:
    /**
     * @brief Initializer
     * @param segPath Vector of points to be segmented
     * @param volpkg Where the segmentation is located
     * @param gravity_scale Limits effect of normals
     * @param threshold How frequently to sample segmentation
     * @param endOffset Last slice to be segmented
     */
    Chain(
        std::vector<cv::Vec3d> segPath,
        const VolumePkg& volpkg,
        double gravity_scale,
        int threshold,
        int endOffset,
        double spring_constant_k = -0.5);
    /**
     * @brief Defines how particles are updated
     */
    void step();
    /** @brief Determines if any of the particles are still moving*/
    bool isMoving();
    /**
     * @brief Returns vector offset that tries to maintain distance between
     * particles
     *
     * This offset is saved in _spring_resting_x.
     *
     * @newline
     *
     * The spring equation (Hooke's law) is -kx where
     * k is the spring constant (stiffness)
     * x is displacement from rest (starting distance between points)
     *
     * @newline
     *
     * There are two if blocks to account for the first and last particles in
     * the chain only having one neighbor.
     *
     */
    cv::Vec3d springForce(int);

    /**
     * @brief Project a vector onto the plane described by the structure
     * tensor-computed normals
     */
    cv::Vec3d gravity(int);

    /** @brief Converts Chain's history into an Ordered Point Set */
    volcart::OrderedPointSet<cv::Vec3d> orderedPCD();

private:
    /** Where the segmentation is stored*/
    const VolumePkg& _volpkg;

    /** History of the chain at each iteration */
    std::list<std::vector<Particle>> _history;
    /** Parameter for calculating the spring effects */
    double _spring_constant_k;
    /** Parameter for calculating the spring effects*/
    double _spring_resting_x;
    /**Limits the effect of the normal vector */
    double _gravity_scale;  // To-Do: Rename.

    /** @name Chain Size Information */
    //@{
    /** Number of particles in the chain & width of output PCD */
    int _chain_length;
    /** Height of the output PCD */
    int _real_iterations;  // To-Do: Do we need this?
    /**Starting slice index */
    int _start_index;
    /** Target slice index */
    int _target_index;
    /** How frequently to sample the segmentation */
    int _threshold;  // To-Do: What is this for now? We may not need this.
    //@}
};
