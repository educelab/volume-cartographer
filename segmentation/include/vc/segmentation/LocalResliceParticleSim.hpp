#pragma once

/** @file */

#include <iostream>

#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/segmentation/ChainSegmentationAlgorithmBaseClass.hpp"
#include "vc/segmentation/lrps/Common.hpp"
#include "vc/segmentation/lrps/FittedCurve.hpp"

namespace volcart::segmentation
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
 * The ending index is inclusive.
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
    /** Default destructor */
    ~LocalResliceSegmentation() override = default;
    /**@}*/

    /**@{*/
    /** @brief Set the target z-index */
    void setTargetZIndex(int z) { endIndex_ = z; }

    /** @brief Set the number of curve optimization iterations per step */
    void setOptimizationIterations(int n) { numIters_ = n; }

    /**
     * @brief Set the weight for the Active Contour metric
     * @see double ActiveContourInternal()
     */
    void setAlpha(double a) { alpha_ = a; }

    /**
     * @brief Set the stretch weight factor
     * @see double ActiveContourInternal()
     */
    void setK1(double k) { k1_ = k; }

    /**
     * @brief Set the curvature weight factor
     * @see double ActiveContourInternal()
     */
    void setK2(double k) { k2_ = k; }

    /**
     * @brief Set the weight for the Absolute Curvature Sum metric
     * @see double AbsCurvatureSum()
     */
    void setBeta(double b) { beta_ = b; }

    /**
     * @brief Set the weight for the Arc Length metric
     * @see double WindowedArcLength()
     */
    void setDelta(double d) { delta_ = d; }

    /**
     * @brief Set the estimated thickness of the substrate (in um)
     *
     * Used to generate the radius of the structure tensor calculation
     */
    void setMaterialThickness(double m) { materialThickness_ = m; }

    /** @brief Set the reslice window size */
    void setResliceSize(int s) { resliceSize_ = s; }

    /** @brief Set the distance weight factor for candidate positions */
    void setDistanceWeightFactor(int f) { peakDistanceWeight_ = f; }

    /** @brief Set whether to consider previous position as candidate position
     */
    void setConsiderPrevious(bool b) { considerPrevious_ = b; }
    /**@}*/

    /**@{*/
    /** @brief Compute the segmentation */
    PointSet compute() override;
    /**@}*/

    /**@{*/
    /** Debug: Shows intensity maps in GUI window */
    void setVisualize(bool b) { visualize_ = b; }

    /** Debug: Dumps reslices and intensity maps to disk */
    void setDumpVis(bool b) { dumpVis_ = b; }
    /**@}*/

    /** @brief Returns the maximum progress value */
    size_t progressIterations() const override;

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

    /** @brief Convert the internal storage array into a final PointSet */
    PointSet create_final_pointset_(
        const std::vector<std::vector<Voxel>>& points);

    /** Default minimum energy gradient */
    constexpr static double DEFAULT_MIN_ENERGY_GRADIENT = 1e-7;

    /** Target z-index */
    int endIndex_{0};
    /** Active Contour weight parameter */
    double alpha_{1.0 / 3.0};
    /** Active Contour stretch parameter */
    double k1_{0.5};
    /** Active Contour curvature parameter */
    double k2_{0.5};
    /** Abs. Curvature Sum weight parameter */
    double beta_{1.0 / 3.0};
    /** Arc Length weight parameter */
    double delta_{1.0 / 3.0};
    /** Distance weight factor for candidate positions */
    int peakDistanceWeight_{50};
    /** Reconsider previous position flag */
    bool considerPrevious_{false};
    /** Dump visualization to disk flag */
    bool dumpVis_{false};
    /** Show visualization in GUI flag */
    bool visualize_{false};
    /** Number of curve optimization iterations */
    int numIters_{15};
    /** Estimated material thickness in um */
    double materialThickness_{100};
    /** Window size for reslice */
    int resliceSize_{32};
};
}  // namespace volcart::segmentation