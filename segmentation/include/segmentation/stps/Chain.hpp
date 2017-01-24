#pragma once

#define DEFAULT_OFFSET (-1)

#include <list>
#include <vector>

#include <opencv2/core.hpp>

#include "core/types/VolumePkg.hpp"
#include "segmentation/stps/Particle.hpp"
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
     * @param gravityScale Limits effect of normals
     * @param threshold How frequently to sample segmentation
     * @param endOffset Last slice to be segmented
     * @param springConstantK Constant factor between springs
     */
    Chain(
        std::vector<cv::Vec3d> segPath,
        const VolumePkg& volpkg,
        double gravityScale,
        int threshold,
        int endOffset,
        double springConstantK = -0.5);
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
    cv::Vec3d springForce(size_t index);

    /**
     * @brief Project a vector onto the plane described by the structure
     * tensor-computed normals
     */
    cv::Vec3d gravity(size_t index);

    /** @brief Converts Chain's history into an Ordered Point Set */
    volcart::OrderedPointSet<cv::Vec3d> orderedPCD();

private:
    /** Where the segmentation is stored*/
    const VolumePkg& volpkg_;

    /** History of the chain at each iteration */
    std::list<std::vector<Particle>> history_;
    /** Parameter for calculating the spring effects */
    double springConstantK_;
    /** Parameter for calculating the spring effects*/
    double springRestingX_;
    /**Limits the effect of the normal vector */
    double gravityScale_;

    /** @name Chain Size Information */
    //@{
    /** Number of particles in the chain & width of output PCD */
    size_t chainLength_;
    /** Height of the output PCD */
    size_t realIterations_;
    /**Starting slice index */
    size_t startIndex_;
    /** Target slice index */
    size_t targetIndex_;
    /** How frequently to sample the segmentation */
    int threshold_;  // To-Do: What is this for now? We may not need this.
    //@}
};
