#pragma once

/** @file */

#include <limits>

#include "vc/core/types/Mixins.hpp"
#include "vc/core/types/PointSet.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/types/VolumetricMask.hpp"

namespace volcart::segmentation
{

/**
 * @brief Compute a VolumetricMask from a PointSet
 *
 * This class uses the flood fill algorithm from ThinnedFloodFillSegmentation to
 * compute a per-voxel mask for a segmented layer in a volume. For each slice
 * in the Z-range of the input PointSet, the points which intersect that slice
 * are used as the seeds for running the flood fill algorithm.
 */
class ComputeVolumetricMask : public IterationsProgress
{
public:
    /** PointSet type */
    using PointSet = volcart::PointSet<cv::Vec3d>;

    /** @brief Set the input PointSet */
    void setPointSet(const PointSet& ps);

    /** @brief Set the input Volume */
    void setVolume(const Volume::Pointer& v);

    /** @brief Set the low threshold for the bounded flood-fill operation. */
    void setLowThreshold(uint16_t t);

    /** @brief Set the high threshold for the bounded flood-fill operation. */
    void setHighThreshold(uint16_t t);

    /** @brief If enabled, apply a morphological closing kernel to the mask */
    void setEnableClosing(bool b);

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

    /**
     * @brief Set the max radius that a single point can hav when measuring the
     * width of the page.
     */
    void setMaxRadius(size_t radius);

    /** @brief Computes the segmentation. */
    VolumetricMask::Pointer compute();

    /** @brief Return the full, 3D mask. */
    VolumetricMask::Pointer getMask() const;

    /** @brief Returns the maximum progress value */
    size_t progressIterations() const override;

private:
    /** Input points */
    PointSet input_;
    /** Input volume */
    Volume::Pointer vol_;
    /** Low flood-fill threshold parameter */
    uint16_t low_{14135};
    /** High flood-fill threshold parameter */
    uint16_t high_{65535};
    /** Enable closing */
    bool enableClosing_{true};
    /** (Square) Kernel size parameter for the closing operation */
    int kernel_{5};
    /** Direction of thickness estimation measurement */
    bool measureVertically_{false};
    /** Maximum layer thickness to consider for a single seed point */
    size_t maxRadius_{std::numeric_limits<size_t>::max()};
    /** Mask */
    VolumetricMask::Pointer mask_;
};

}  // namespace volcart::segmentation