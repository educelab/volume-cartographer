#pragma once

#include <iostream>

#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/segmentation/ChainSegmentationAlgorithmBaseClass.hpp"
#include "vc/segmentation/lrps/Common.hpp"
#include "vc/segmentation/lrps/FittedCurve.hpp"

namespace volcart
{
namespace segmentation
{
/**
 * @class LocalResliceSegmentation
 * @brief Local Reslice %Particle Simulation (LRPS) segmentation
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
class LocalResliceSegmentation : public ChainSegmentationAlgorithmBaseClass
{
public:
    /** @name Constructors */
    /**@{*/
    /** @brief Default constructor */
    LocalResliceSegmentation() = default;
    /**@}*/

    //    /**
    //     * @brief Run LRPS with the provided path
    //     * @param cloud Starting chain
    //     * @param startIndex Starting z-index
    //     * @param endIndex Ending z-index
    //     * @param numIters Number of curve optimization iterations per step
    //     * @param step Number of z-indices to move each iteration
    //     * @param alpha Used to calculate energy
    //     * @param k1 Used to calculate energy
    //     * @param k2 Used to calculate energy
    //     * @param beta Used to calculate energy
    //     * @param delta Used to calculate energy
    //     * @param peakDistanceWeight Distance weight factor for maxima
    //     * @param shouldIncludeMiddle Include previous position as candidate
    //     new
    //     * position
    //     * @param dumpVis Debug: Write reslices and IntensityMaps to disk
    //     * @param visualize Debug: Show IntensityMaps in GUI window
    //     * @return Segmentation surface point set
    //     */
    PointSet compute() override;

    void setTargetZIndex(int z) { endIndex_ = z; }
    void setOptimizationIterations(int n) { numIters_ = n; }

    void setAlpha(double a) { alpha_ = a; }
    void setK1(double k) { k1_ = k; }
    void setK2(double k) { k2_ = k; }

    void setBeta(double b) { beta_ = b; }

    void setDelta(double d) { delta_ = d; }

    /** @brief Set the reslice window size */
    void setResliceSize(int s) { resliceSize_ = s; }
    void setDistanceWeightFactor(int f) { peakDistanceWeight_ = f; }

    void setConsiderPrevious(bool b) { considerPrevious_ = b; }
    void setVisualize(bool b) { visualize_ = b; }
    void setDumpVis(bool b) { dumpVis_ = b; }

    /** @brief Set normal estimation radius */
    void setMaterialThickness(double m) { materialThickness_ = m; }

private:
    /**
     * @brief Estimate the normal to the curve at point index
     * @param currentCurve Input curve
     * @param index Index of point on curve
     */
    cv::Vec3d estimate_normal_at_index_(
        const FittedCurve& currentCurve, int index);

    /**
     * @brief Debug: Draw curve on slice image
     * @param curve Input curve
     * @param sliceIndex %Slice on which to draw
     * @param particleIndex Highlight point at particleIndex
     * @param showSpline Draw interpolated curve. Default only draws points
     */
    cv::Mat draw_particle_on_slice_(
        const FittedCurve& curve,
        int sliceIndex,
        int particleIndex = -1,
        bool showSpline = false) const;

    PointSet create_final_pointset_(
        const std::vector<std::vector<Voxel>>& points);

    /** Default minimum energy gradient */
    constexpr static double DEFAULT_MIN_ENERGY_GRADIENT = 1e-7;

    int endIndex_{0};
    double alpha_{1.0 / 3.0};
    double k1_{0.5};
    double k2_{0.5};
    double beta_{1.0 / 3.0};
    double delta_{1.0 / 3.0};
    int peakDistanceWeight_{50};
    bool considerPrevious_{false};
    bool dumpVis_{false};
    bool visualize_{false};
    int numIters_{15};
    double materialThickness_{100};
    /** Window size for reslice */
    int resliceSize_{32};
};
}
}