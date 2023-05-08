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
 * @brief Multithreaded Optical Flow Segmentation
 *
 * This algorithm propagates a chain of points forward through a volume from
 * a starting z-index to an ending z-index. Each point is assumed to start
 * within a page layer. 
 * The ending index is inclusive.
 *
 * Warning: This Algorithm is not deterministic and yields slightly different results each run.
 *
 * @ingroup ofsc
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
     * @brief Set the threshold of what pixel brightness is considered inside a sheet (higher as threshold) and outside (lower as threshold)
     */
    void setOutsideThreshold(int outside) { outside_threshold_ = outside; }

    /**
     * @brief Set the threshold of what pixel brightness is considered while calculating optical flow, darker pixels OF is interpolated from brighter ones in the area
     */
    void setOFThreshold(int of_thr) { optical_flow_pixel_threshold_ = of_thr; }

    /**
     * @brief Set the estimated thickness of the substrate (in um)
     *
     * Used to generate the radius of the structure tensor calculation
     */
    void setMaterialThickness(double m) { materialThickness_ = m; }

    /** @brief Set the maximum single pixel optical flow displacement before interpolating a pixel region */
    void setOFDispThreshold(int of_disp_thrs) { optical_flow_displacement_threshold_ = of_disp_thrs; }

    /** @brief Set whether to purge cache
     */
    void setPurgeCache(bool purge_c) { purge_cache_ = purge_c; }
    /**@}*/

    /** @brief Set how many slices should be cached
     */
    void setCacheSlices(int cache_slices) { nr_cache_slices_ = cache_slices; }

    /** @brief Set how many slices should be cached
     */
    void setLineSmoothenByBrightness(int brightness) { smoothen_by_brightness_ = brightness; }

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
    /** Darker pixels are considered outside the sheet */
    int outside_threshold_{80};
    /** Disregarding pixel that are darker during optical flow computation */
    int optical_flow_pixel_threshold_{80};
    /** Threshold of how many pixel optical flow can displace a point, if higher, recompute optical flow with region's average flow */
    int optical_flow_displacement_threshold_{10};
    /** Purging Cache flag */
    bool purge_cache_{true};
    /** Dump visualization to disk flag */
    bool dumpVis_{false};
    /** Show visualization in GUI flag */
    bool visualize_{false};
    /** Number of curve optimization iterations */
    int numIters_{15};
    /** Estimated material thickness in um */
    double materialThickness_{100};
    /** Number of slices to cache */
    int nr_cache_slices_{-1};
    /** Number of slices to cache */
    int smoothen_by_brightness_{180};
    
};
}  // namespace volcart::segmentation