#pragma once

/** @file */

#include <cstddef>
#include <cstdint>
#include <optional>

#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/segmentation/ChainSegmentationAlgorithm.hpp"
#include "vc/segmentation/lrps/FittedCurve.hpp"

namespace volcart::segmentation
{
/**
 * @brief Optical Flow Segmentation
 *
 * @author    Julian Schilliger
 * @date      May 2023
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
class OpticalFlowSegmentation : public ChainSegmentationAlgorithm
{
public:
    /** Pointer */
    using Pointer = std::shared_ptr<OpticalFlowSegmentation>;

    /** @brief Default constructor */
    OpticalFlowSegmentation() = default;

    /** Default destructor */
    ~OpticalFlowSegmentation() override = default;

    /** Make a new shared instance */
    template <typename... Args>
    static auto New(Args... args) -> Pointer
    {
        return std::make_shared<OpticalFlowSegmentation>(
            std::forward<Args>(args)...);
    }

    /** @brief Set the start z-index */
    void setStartZIndex(int z);

    /** @brief Get the start z-index */
    [[nodiscard]] auto getStartZIndex() const -> int;

    /** @brief Set the target z-index */
    void setTargetZIndex(int z);

    /** @brief Get the target z-index */
    [[nodiscard]] auto getTargetZIndex() const -> int;

    /**
     * @brief Set the threshold of what pixel brightness is considered inside a
     * sheet (higher as threshold) and outside (lower as threshold)
     */
    void setOutsideThreshold(std::uint8_t outside);

    /**
     * @brief Set the threshold of what pixel brightness is considered while
     * calculating optical flow, darker pixels OF is interpolated from brighter
     * ones in the area
     */
    void setOFThreshold(std::uint8_t ofThr);

    /**
     * @brief Set the maximum single pixel optical flow displacement before
     * interpolating a pixel region
     */
    void setOFDispThreshold(std::uint32_t ofDispThrs);

    /**
     * @brief Set the threshold for what pixel brightness is considered as
     * being outside the sheet. Pixels above this threshold are considered
     * outside the sheet and are smoothed in an attempt to get them tracking
     * the sheet again.
     */
    void setSmoothBrightnessThreshold(std::uint8_t brightness);

    /** @brief Set whether to enable outlier points smoothening */
    void setEnableSmoothOutliers(bool enable);

    /** @brief Set whether to enable edge detection */
    void setEnableEdgeDetection(bool enable);

    /** @brief Set the minimum jump distance for edge detection */
    void setEdgeJumpDistance(std::uint32_t distance);

    /** @brief Set the maximum bounce distance for edge detection */
    void setEdgeBounceDistance(std::uint32_t distance);

    /** @brief Set whether to interpolate against the master cloud */
    void setInterpolate(bool b);

    /** @brief Set how wide the interpolation window should be
     */
    void setInterpolationWindow(std::uint32_t window);

    /** @brief Get how wide the interpolation window should be
     */
    [[nodiscard]] auto getInterpolationWindow() const -> std::uint32_t;

    /** @brief Set how many slices the interpolation center is away from the
     * start slice
     */
    void setInterpolationDistance(std::uint32_t distance);

    /** @brief Get how many slices the interpolation center is away from the
     * start slice
     */
    [[nodiscard]] auto getInterpolationDistance() const -> std::uint32_t;

    /**
     * @brief Set the already computed masterCloud OrderedPointSet
     */
    void setMasterCloud(PointSet masterCloud);

    /** @brief Set the input chain of re-segmentation points */
    void setReSegmentationChain(Chain c);

    /**
     * @brief Set the estimated thickness of the substrate (in um)
     *
     * Used to generate the radius of the structure tensor calculation
     */
    void setMaterialThickness(double m);

    /** @brief Set the maximum number of threads */
    void setMaxThreads(std::uint32_t t);

    /** @brief Clear the maximum number of threads */
    void resetMaxThreads();

    /** Debug: Shows intensity maps in GUI window */
    void setVisualize(bool b);

    /** Debug: Dumps reslices and intensity maps to disk */
    void setDumpVis(bool b);

    /** @brief Compute the segmentation */
    auto compute() -> PointSet override;

    /** @brief Returns the maximum progress value */
    [[nodiscard]] auto progressIterations() const -> std::size_t override;

private:
    /** @brief Configuration for a single run_ofs_() invocation */
    struct OfsConfig {
        /** True to propagate toward lower z-indices */
        bool backwards{false};
        /** True to prepend each new row rather than append */
        bool insertFront{false};
        /** Directory for per-slice debug visualizations */
        filesystem::path outputDir;
        /** Directory for whole-chain debug visualizations */
        filesystem::path wholeChainDir;
    };

    /**
     * @brief Compute the curve for z + 1 given a curve on z using the optical
     * flow between the two slices
     */
    [[nodiscard]] auto compute_curve_(
        const FittedCurve& currentCurve, int zIndex) const
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

    /**
     * @brief Blend the re-segmentation run into the forward run over an
     * interpolation window
     * @param interpStart First z-slice of the interpolation window
     * @param interpEnd Last z-slice of the interpolation window
     * @param startChain Z-index of the primary starting chain
     * @param startResegChain Z-index of the re-segmentation starting chain
     */
    auto interpolate_(
        int interpStart, int interpEnd, int startChain, int startResegChain)
        -> std::vector<std::vector<Voxel>>;

    /**
     * @brief Run the optical flow segmentation in one direction
     * @param currentVs Starting chain of points
     * @param startChainIndex Z-index of the starting chain
     * @param anchorEndIdx Z-index at which to stop and return
     * @param targetIndex Final target z-index (used for bounds checking)
     * @param stepAdjustment Initial step offset to align with the grid
     * @param iteration Running progress counter (updated in place)
     * @param cfg Direction, insertion order, and debug-output settings
     */
    auto run_ofs_(
        Chain currentVs,
        int startChainIndex,
        int anchorEndIdx,
        int targetIndex,
        int stepAdjustment,
        std::size_t& iteration,
        const OfsConfig& cfg)
        -> std::tuple<std::vector<std::vector<Voxel>>, Status>;

    /**
     * @brief Interpolate a set of curve rows against the master cloud
     * @param points Curve rows to blend
     * @param windowSize Number of rows on each side to blend
     * @param backwards Direction of the OFS run
     */
    auto interpolateWithMasterCloud(
        std::vector<std::vector<Voxel>> points, int windowSize, bool backwards)
        -> std::vector<std::vector<Voxel>>;

    /**
     * @brief Fill in missing z-slices between curve rows using linear
     * interpolation
     */
    auto interpolateGaps(std::vector<std::vector<Voxel>> points)
        -> std::vector<std::vector<Voxel>>;

    /** Start z-index */
    int startIndex_{0};
    /** Target z-index */
    int endIndex_{0};
    /**
     * Darker pixels are considered outside the sheet. This parameter sets the
     * threshold of what pixel brightness is considered too deep inside a sheet
     * (higher than the threshold) and then tries to smoothen those points back
     * towards the edge of the sheet. Range: 0-255.
     */
    std::uint8_t outsideThreshold_{80};
    /**
     * Disregarding pixel that are darker during optical flow computation. This
     * parameter sets the threshold for what pixel brightness is considered
     * while calculating optical flow. Darker pixels' optical flow is
     * interpolated from brighter ones in the area. Range: 0-255. Higher values
     * disregard more dark pixels during computation, while lower values
     * include more dark pixels.
     */
    std::uint8_t opticalFlowPixelThreshold_{80};
    /**
     * Threshold of how many pixels optical flow can displace a point, if
     * higher, recompute optical flow with region's average flow. This
     * parameter sets the maximum single pixel optical flow displacement before
     * interpolating a pixel region. Range minimum: 0. Higher values allow more
     * displacement before interpolation, while lower values trigger
     * interpolation more frequently.
     */
    std::uint32_t opticalFlowDisplacementThreshold_{10};
    /**
     * This parameter sets the threshold for what pixel brightness is considered
     * as being outside the sheet. Pixels considered outside the sheet are
     * smoothed in an attempt to get them tracking the sheet again.
     * Range: 0-255. Smooth curve at pixels above this threshold.
     */
    std::uint8_t smoothByBrightness_{180};
    /** Enable smoothing of detected outlier points */
    bool enableSmoothenOutlier_{true};
    /** Enable edge detection to constrain point movement */
    bool enableEdge_{false};
    /** Minimum displacement (voxels) considered an edge jump */
    std::uint32_t edgeJumpDistance_{6};
    /** Maximum displacement (voxels) allowed as a bounce from an edge */
    std::uint32_t edgeBounceDistance_{3};
    /** Enable blending of OFS output against the master cloud */
    bool requestInterp_{true};
    /** Half-width of the interpolation window in slices (must be positive) */
    std::uint32_t interpWindow_{5};
    /** Distance in slices from the start slice to the interpolation center */
    std::uint32_t interpDist_{25};
    /** Re-segmentation starting chain provided by the caller */
    Chain resegStartingChain_;
    /** Pre-computed master cloud used for interpolation blending */
    PointSet masterCloud_;
    /** Estimated material thickness in um */
    double materialThickness_{100};
    /** Maximum number of threads */
    std::optional<std::uint32_t> maxThreads_;
    /** Dump visualization to disk flag */
    bool dumpVis_{false};
    /** Show visualization in GUI flag */
    bool visualize_{false};
};
}  // namespace volcart::segmentation