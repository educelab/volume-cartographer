#pragma once

#include <limits>

#include "vc/segmentation/RegionGrowingSegmentationAlgorithmBaseClass.hpp"

namespace volcart
{
namespace segmentation
{
/**
 * @brief ThinnedFloodFillSegmentation
 *
 * The algorithm begins with a user-defined set of seed points. The estimated
 * median thickness of the page is calculated from each seed point. A flood-fill
 * operation begins at each seed point, extending outwards to neighboring
 * pixels--within the boundary imposed on the original seed point by the
 * estimated median thickness--if the neighboring pixels are within the
 * user-specified grey threshold. The mask that each seed point creates overlaps
 * with the other masks, creating a full-thickness segmentation of the page. The
 * mask is then thinned to a continuous (as much as possible) skeleton that is
 * centered inside the thickness of the page. Any spurs are removed from the
 * skeleton. All the points in the skeleton are used as seed points for the next
 * slice. Using this approach results in a full-thickness segmentation of the
 * page for each slice.
 *
 * Note: This algorithm operates solely on two-dimensional slices and does not
 * use any 3D subvolumes.
 *
 * @ingroup Segmentation
 */
class ThinnedFloodFillSegmentation
    : public RegionGrowingSegmentationAlgorithmBaseClass
{
public:
    using VoxelMask = volcart::PointSet<cv::Vec3i>;

    /** Sends when the segmentation is updated with intermediate results */
    Signal<PointSet> pointsetUpdated;
    /** Sends when the layer mask is updated with intermediate results */
    Signal<VoxelMask> maskUpdated;

    /** @brief Default constructor */
    ThinnedFloodFillSegmentation() = default;
    /** Default destructor */
    ~ThinnedFloodFillSegmentation() override = default;
    /** Default copy constructor */
    ThinnedFloodFillSegmentation(const ThinnedFloodFillSegmentation&) = default;
    /** Default copy operator */
    ThinnedFloodFillSegmentation& operator=(
        const ThinnedFloodFillSegmentation&) = default;
    /** Default move constructor */
    ThinnedFloodFillSegmentation(ThinnedFloodFillSegmentation&&) = default;
    /** Default move operator */
    ThinnedFloodFillSegmentation& operator=(ThinnedFloodFillSegmentation&&) =
        default;

    /** @brief Set the low threshold for the bounded flood-fill operation. */
    void setFFLowThreshold(uint16_t t);

    /** @brief Set the high threshold for the bounded flood-fill operation. */
    void setFFHighThreshold(uint16_t t);

    /**
     * @brief Set the threshold for the distance transform pre-processing
     * operation.
     */
    void setDistanceTransformThreshold(float t);

    /**
     * @brief Set the kernel size used in the closing operation. Larger kernels
     * close bigger gaps.
     */
    void setClosingKernelSize(int s);

    /**
     * @brief Set the direction in which the thickness of the page will be
     * measured. If this value is false, the thickness will be measured
     * horizontally (+/- x)
     */
    void setMeasureVertical(bool b);

    /** @brief Set the max length of spurs that can be pruned.  */
    void setSpurLengthThreshold(int length);

    /**
     * @brief Set the max radius that a single point can hav when measuring the
     * width of the page.
     */
    void setMaxRadius(size_t radius);

    /** @brief Computes the segmentation. */
    PointSet compute() override;

    /** @brief Return the full, 3D mask. */
    VoxelMask getMask() const;

    /**
     * @brief Debug: Dumps visualizations of the mask and skeleton for each
     * slice to disk.
     *
     * Images are saved in the 'dumpvis' folder, created relative to the working
     * directory.
     */
    void setDumpVis(bool b);

private:
    /** Low flood-fill threshold parameter */
    uint16_t low_{14135};
    /** High flood-fill threshold parameter */
    uint16_t high_{65535};
    /** Distance transform threshold parameter */
    float dtt_{0};
    /** (Square) Kernel size parameter for the closing operation */
    int kernel_{5};
    /** Dump visualization to disk flag */
    bool dumpVis_{false};
    /** Prune spurs of this length or shorter */
    size_t spurLength_{6};
    /**
     * Indicate which direction thickness should be measured in.
     * Horizontal by default
     */
    bool measureVertically_{false};
    /** Maximum layer thickness to consider for a single seed point */
    size_t maxRadius_{std::numeric_limits<size_t>::max()};
    /** Mask */
    VoxelMask volMask_;
    /** Measure the thickness of the layer relative to a seed */
    size_t measure_thickness_(
        const cv::Vec3i& seed, const cv::Mat& slice) const;
};
}  // namespace segmentation
}  // namespace volcart
