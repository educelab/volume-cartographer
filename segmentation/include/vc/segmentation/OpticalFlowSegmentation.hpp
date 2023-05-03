#pragma once

/** @file */

#include <iostream>

#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/segmentation/ChainSegmentationAlgorithm.hpp"
#include "vc/segmentation/lrps/Common.hpp"
#include "vc/segmentation/lrps/FittedCurve.hpp"

namespace volcart::segmentation
{
/**
 * @class OpticalFlowSegmentationClass
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
class OpticalFlowSegmentationClass : public ChainSegmentationAlgorithm
{
public:
    /** Pointer */
    using Pointer = std::shared_ptr<OpticalFlowSegmentationClass>;

    /** @brief Default constructor */
    OpticalFlowSegmentationClass() = default;

    /** Default destructor */
    ~OpticalFlowSegmentationClass() override = default;

    /** Make a new shared instance */
    template <typename... Args>
    static auto New(Args... args) -> Pointer
    {
        return std::make_shared<OpticalFlowSegmentationClass>(
            std::forward<Args>(args)...);
    }

    /** @brief Set the target z-index */
    void setTargetZIndex(int z) { endIndex_ = z; }

    /** @brief Set the number of curve optimization iterations per step */
    void setOptimizationIterations(int n) { numIters_ = n; }

    /**
     * @brief Set the weight for the Active Contour metric
     * @see double ActiveContourInternal()
     */
    void setOutsideThreshold(int outside) { outside_threshold_ = outside; }

    /**
     * @brief Set the weight for the Absolute Curvature Sum metric
     * @see double AbsCurvatureSum()
     */
    void setOFThreshold(int of_thr) { optical_flow_pixel_threshold_ = of_thr; }

    /**
     * @brief Set the estimated thickness of the substrate (in um)
     *
     * Used to generate the radius of the structure tensor calculation
     */
    void setMaterialThickness(double m) { materialThickness_ = m; }

    /** @brief Set the distance weight factor for candidate positions */
    void setOFDispThreshold(int of_disp_thrs) { optical_flow_displacement_threshold_ = of_disp_thrs; }

    /** @brief Set whether to consider previous position as candidate position
     */
    void setPurgeCache(bool purge_c) { purge_cache_ = purge_c; }
    /**@}*/

    /**@{*/
    /** @brief Compute the segmentation 1 Line */
    std::vector<Voxel> computeCurve(FittedCurve currentCurve, Chain& currentVs, int zIndex);
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

    /** @brief Returns the maximum progress value */
    [[nodiscard]] auto progressIterations() const -> size_t override;

private:
    /**
     * @brief Estimate the normal to the curve at point index
     * @param currentCurve Input curve
     * @param index Index of point on curve
     */
    auto estimate_normal_at_index_(const FittedCurve& currentCurve, int index)
        -> cv::Vec3d;

    /**
     * @brief Estimate the 2D normal to the curve at point index in the z plane
     * @param curve Input curve
     * @param index Index of point on curve
     */
    auto estimate_2d_normal_at_index_(const volcart::segmentation::FittedCurve& curve, int index)
        -> cv::Vec2f;

    /**
     * @brief Calculate the mean pixel value in a window around a point
     * @param integral_img Intergral image of slice
     * @param pt Point around which to calculate mean
     * @param window_size Size of window
     */
    auto get_mean_pixel_value(const cv::Mat& integral_img, const cv::Point& pt, int window_size)
        -> float;

    /**
     * @brief Debug: Draw curve on slice image
     * @param curve Input curve
     * @param sliceIndex %Slice on which to draw
     * @param particleIndex Highlight point at particleIndex
     * @param showSpline Draw interpolated curve. Default only draws points
     */
    [[nodiscard]] auto draw_particle_on_slice_(
        const FittedCurve& curve,
        int sliceIndex,
        int particleIndex = -1,
        bool showSpline = false) const -> cv::Mat;

    /** @brief Convert the internal storage array into a final PointSet */
    auto create_final_pointset_(const std::vector<std::vector<Voxel>>& points)
        -> PointSet;

    /** Default minimum energy gradient */
    constexpr static double DEFAULT_MIN_ENERGY_GRADIENT = 1e-7;

    /** Target z-index */
    int endIndex_{0};
    /** Active Contour weight parameter */
    int outside_threshold_{80};
    /** Abs. Curvature Sum weight parameter */
    int optical_flow_pixel_threshold_{80};
    /** Distance weight factor for candidate positions */
    int optical_flow_displacement_threshold_{10};
    /** Reconsider previous position flag */
    bool purge_cache_{true};
    /** Dump visualization to disk flag */
    bool dumpVis_{false};
    /** Show visualization in GUI flag */
    bool visualize_{false};
    /** Number of curve optimization iterations */
    int numIters_{15};
    /** Estimated material thickness in um */
    double materialThickness_{100};
};
}  // namespace volcart::segmentation