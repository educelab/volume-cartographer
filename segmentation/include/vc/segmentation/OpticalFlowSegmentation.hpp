#pragma once

/** @file */

#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/segmentation/ChainSegmentationAlgorithm.hpp"
#include "vc/segmentation/lrps/FittedCurve.hpp"

namespace volcart::segmentation
{
/**
 * @brief Optical Flow Segmentation
 *
 * @author Julian Schilliger (May 2023)
 *
 * This algorithm propagates a chain of points forward through a volume from a
 * starting z-index to an ending z-index (inclusive). It uses optical flow to
 * track the shape of the layer. Each seed point is assumed to be placed within
 * the layer rather than on its surface boundary.
 *
 * @warning This algorithm is non-deterministic and yields slightly different
 * results each run.
 *
 * @ingroup Segmentation
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
    void setTargetZIndex(int z);

    /**
     * @brief Set the threshold of what pixel brightness is considered inside a
     * sheet (higher as threshold) and outside (lower as threshold)
     */
    void setOutsideThreshold(int outside);

    /**
     * @brief Set the threshold of what pixel brightness is considered while
     * calculating optical flow, darker pixels OF is interpolated from brighter
     * ones in the area
     */
    void setOFThreshold(int ofThr);

    /**
     * @brief Set the maximum single pixel optical flow displacement before
     * interpolating a pixel region
     */
    void setOFDispThreshold(int ofDispThrs);

    /** @brief Set how many slices should be cached */
    void setLineSmoothenByBrightness(int brightness);

    /**
     * @brief Set the estimated thickness of the substrate (in um)
     *
     * Used to generate the radius of the structure tensor calculation
     */
    void setMaterialThickness(double m);

    /** Debug: Shows intensity maps in GUI window */
    void setVisualize(bool b);

    /** Debug: Dumps reslices and intensity maps to disk */
    void setDumpVis(bool b);

    /** @brief Compute the segmentation */
    auto compute() -> PointSet override;

    /** @brief Returns the maximum progress value */
    [[nodiscard]] auto progressIterations() const -> size_t override;

private:
    /**
     * @brief Estimate the 2D normal to the curve at point index in the z plane
     * @param curve Input curve
     * @param index Index of point on curve
     */
    auto estimate_2d_normal_at_index_(const FittedCurve& curve, int index)
        -> cv::Vec2f;

    /**
     * @brief Calculate the mean pixel value in a window around a point
     * @param integralImg Intergral image of slice
     * @param pt Point around which to calculate mean
     * @param windowSize Size of window
     */
    auto get_mean_pixel_value_(
        const cv::Mat& integralImg, const cv::Point& pt, int windowSize)
        -> float;

    /** @brief Compute the segmentation 1 Line */
    auto compute_curve_(const FittedCurve& currentCurve, int zIndex)
        -> std::vector<Voxel>;

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

    /** Target z-index */
    int endIndex_{0};
    /**
     * Darker pixels are considered outside the sheet. This parameter sets the
     * threshold of what pixel brightness is considered too deep inside a sheet
     * (higher than the threshold) and then tries to smoothen those points back
     * towards the edge of the sheet. Range: 0-255.
     */
    int outsideThreshold_{80};
    /**
     * Disregarding pixel that are darker during optical flow computation. This
     * parameter sets the threshold for what pixel brightness is considered
     * while calculating optical flow. Darker pixels' optical flow is
     * interpolated from brighter ones in the area. Range: 0-255. Higher values
     * disregard more dark pixels during computation, while lower values
     * include more dark pixels.
     */
    int opticalFlowPixelThreshold_{80};
    /**
     * Threshold of how many pixel optical flow can displace a point, if
     * higher, recompute optical flow with region's average flow. This
     * parameter sets the maximum single pixel optical flow displacement before
     * interpolating a pixel region. Range minimum: 0. Higher values allow more
     * displacement before interpolation, while lower values trigger
     * interpolation more frequently.
     */
    int opticalFlowDisplacementThreshold_{10};
    /**
     * This parameter sets the threshold for what pixel brightness is considered
     * as being outside the sheet. Pixels considered outside the sheet are
     * smoothed in an attempt to get them tracking the sheet again.
     * Range: 0-256. Smoothen curve at pixels above this threshold.
     */
    int smoothenByBrightness_{180};
    /** Estimated material thickness in um */
    double materialThickness_{100};
    /** Dump visualization to disk flag */
    bool dumpVis_{false};
    /** Show visualization in GUI flag */
    bool visualize_{false};
};
}  // namespace volcart::segmentation