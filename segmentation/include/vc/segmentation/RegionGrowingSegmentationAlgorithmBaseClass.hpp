#pragma once

/** @file */

#include "vc/core/types/Mixins.hpp"
#include "vc/core/types/PointSet.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart::segmentation
{
/**
 * @class RegionGrowingSegmentationAlgorithmBaseClass
 * @brief Base class for segmentation algorithms that create a segmentation by
 * growing from a set of seed points
 *
 * @ingroup Segmentation
 */
class RegionGrowingSegmentationAlgorithmBaseClass : public IterationsProgress
{
public:
    virtual ~RegionGrowingSegmentationAlgorithmBaseClass() = default;
    RegionGrowingSegmentationAlgorithmBaseClass(
        const RegionGrowingSegmentationAlgorithmBaseClass&) = default;
    RegionGrowingSegmentationAlgorithmBaseClass& operator=(
        const RegionGrowingSegmentationAlgorithmBaseClass&) = default;
    RegionGrowingSegmentationAlgorithmBaseClass(
        RegionGrowingSegmentationAlgorithmBaseClass&&) = default;
    RegionGrowingSegmentationAlgorithmBaseClass& operator=(
        RegionGrowingSegmentationAlgorithmBaseClass&&) = default;

    /** Seed points type */
    using SeedPoints = std::vector<cv::Vec3d>;

    /** PointSet type */
    using PointSet = volcart::PointSet<cv::Vec3d>;

    /** Computation result status */
    enum class Status { Success, Failure, ReturnedEarly };

    /** @brief Set the input Volume */
    void setVolume(Volume::Pointer vol) { vol_ = std::move(vol); }

    /** @brief Set the input seed points */
    void setSeedPoints(SeedPoints p) { startingPoints_ = std::move(p); }

    /** @brief Set the number of iterations */
    void setIterations(size_t i) { iterations_ = i; }

    /** @brief Compute the segmentation */
    virtual PointSet compute() = 0;

    /** @brief Get the status of the previous computation */
    Status getStatus() const { return status_; }

    /** @brief Get the segmented points */
    const PointSet& getPointSet() const { return result_; }

    /** @copydoc getPointSet() const */
    PointSet& getPointSet() { return result_; }

    /** @brief Returns the maximum progress value */
    size_t progressIterations() const override { return iterations_; }

protected:
    /** Default constructor */
    RegionGrowingSegmentationAlgorithmBaseClass() = default;

    /** Volume */
    Volume::Pointer vol_;
    /** Seed points */
    SeedPoints startingPoints_;
    /** Result */
    PointSet result_;
    size_t iterations_{0};
    /** Computation status */
    Status status_{Status::Success};
};
}  // namespace volcart::segmentation
