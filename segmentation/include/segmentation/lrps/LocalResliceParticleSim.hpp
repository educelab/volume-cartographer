#pragma once

#include <iostream>

#include "core/types/OrderedPointSet.hpp"
#include "core/types/VolumePkg.hpp"
#include "segmentation/lrps/Common.hpp"
#include "segmentation/lrps/FittedCurve.hpp"

namespace volcart
{
namespace segmentation
{
/**
 * @class LocalResliceSegmentation
 * @brief Local Reslice Particle Simulation (LRPS) segmentation
 *
 * This algorithm propagates a chain of points forward through a volume from
 * a starting z-index to an ending z-index. Each point is assumed to start
 * within a "channel" of maximum intensity, that is, the bright voxels of a
 * page layer. As the chain propagates forward, each point produces a reslice
 * image that is locally orthogonal to the layer. Using this view, the algorithm
 * finds the weighted set of maxima local to the point. These are assumed to be
 * layer voxels on the current or a nearby layer. Using the sorted maxima for
 * each point, the algorithm then selects the propagated positions for all
 * points in the chain that minimize the energy loss of the chain curvature.
 *
 * Starting and ending indexes are inclusive.
 *
 * @ingroup lrps
 */
class LocalResliceSegmentation
{
public:
    //** @name Constructors */
    //@{
    /** @brief Construct with VolumePkg */
    explicit LocalResliceSegmentation(VolumePkg& pkg) : pkg_{pkg} {}
    //@}

    /**
     * @brief Run LRPS with the provided path
     * @param cloud Starting chain
     * @param startIndex Starting z-index
     * @param endIndex Ending z-index
     * @param numIters Number of curve optimization iterations per step
     * @param step Number of z-indices to move each iteration
     * @param alpha Used to calculate energy
     * @param k1 Used to calculate energy
     * @param k2 Used to calculate energy
     * @param beta Used to calculate energy
     * @param delta Used to calculate energy
     * @param peakDistanceWeight Distance weight factor for maxima
     * @param shouldIncludeMiddle Include previous position as candidate new
     * position
     * @param dumpVis Debug: Write reslices and IntensityMaps to disk
     * @param visualize Debug: Show IntensityMaps in GUI window
     * @return Segmentation surface point set
     */
    volcart::OrderedPointSet<cv::Vec3d> segmentPath(
        std::vector<cv::Vec3d> cloud,
        int startIndex,
        int endIndex,
        int numIters,
        int step,
        double alpha,
        double k1,
        double k2,
        double beta,
        double delta,
        int peakDistanceWeight,
        bool shouldIncludeMiddle,
        bool dumpVis,
        bool visualize);

private:
    /** VolumePkg from which to pull intensity information */
    VolumePkg& pkg_;

    /** @brief Estimate the normal to the curve at point index
     * @param currentCurve Input curve
     * @param index Index of point on curve
     */
    cv::Vec3d estimate_normal_at_index_(
        const FittedCurve& currentCurve, int index);

    /** @brief Debug: Draw curve on slice image
     * @param curve Input curve
     * @param sliceIndex Slice on which to draw
     * @param particleIndex Highlight point at particleIndex
     * @param showSpline Draw interpolated curve. Default only draws points
     */
    cv::Mat draw_particle_on_slice_(
        const FittedCurve& curve,
        int sliceIndex,
        int particleIndex = -1,
        bool showSpline = false) const;

    /** Default minimum energy gradient */
    constexpr static double DEFAULT_MIN_ENERGY_GRADIENT = 1e-7;
};
}
}
