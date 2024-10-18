#pragma once

/** @file */

#include <cstddef>

#include "vc/core/types/BoundingBox.hpp"
#include "vc/core/types/Mixins.hpp"
#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart::segmentation
{
/**
 * @class ChainSegmentationAlgorithm
 * @author Seth Parker
 * @brief Base class for segmentation algorithms that propagate a collected
 * chain of points
 */
class ChainSegmentationAlgorithm : public IterationsProgress
{
public:
    /** Shared pointer type */
    using Pointer = std::shared_ptr<ChainSegmentationAlgorithm>;

    /** Default destructor for virtual base class */
    ~ChainSegmentationAlgorithm() override = default;

    /** Chain type */
    using Chain = std::vector<cv::Vec3d>;
    /** PointSet type */
    using PointSet = OrderedPointSet<cv::Vec3d>;
    /** Bounding Box type */
    using Bounds = BoundingBox<double, 3>;
    /** Computation result status */
    enum class Status { Success, Failure, ReturnedEarly };

    /**@{*/
    /** @brief Set the input Volume */
    void setVolume(Volume::Pointer vol)
    {
        vol_ = std::move(vol);
        bb_ = vol_->bounds();
    }

    /**
     * @brief Set the bounding box for computation
     *
     * Used by an derived algorithm to determine a out-of-bounds stop condition
     * for computation.
     */
    void setBounds(Bounds b) { bb_ = std::move(b); }

    /** @brief Set the input chain of seed points */
    void setChain(Chain c) { startingChain_ = std::move(c); }
    /**@}*/

    /**@{*/
    /** @brief Set the number of propagation steps
     *
     * Typically, the expected number of output iterations. Derived algorithms
     * may produce intermediate iterations that are not included in the output.
     */
    void setNumberOfSteps(std::size_t n) { numSteps_ = n; }

    /** @brief Set the propagation step size
     *
     * Typically the expected unit distance between each output iteration in
     * voxel units.
     */
    void setStepSize(double s) { stepSize_ = s; }
    /**@}*/

    /**@{*/
    /** @brief Compute the segmentation */
    virtual auto compute() -> PointSet = 0;

    /** @brief Get the status of the previous computation */
    auto getStatus() const -> Status { return status_; }
    /**@}*/

    /**@{*/
    /** @brief Get the segmented pointset */
    [[nodiscard]] auto getPointSet() const -> const PointSet&
    {
        return result_;
    }

    /** @copydoc getPointSet() const */
    auto getPointSet() -> PointSet& { return result_; }
    /**@}*/

    /** @brief Returns the maximum progress value */
    auto progressIterations() const -> std::size_t override
    {
        return numSteps_;
    }

protected:
    /** Default constructor */
    ChainSegmentationAlgorithm() = default;
    /** Volume */
    Volume::Pointer vol_;
    /** Seed chain */
    Chain startingChain_;
    /** Bounding box */
    Bounds bb_;
    /** Number of propagation steps */
    std::size_t numSteps_{0};
    /** Propagation step size */
    double stepSize_{1.0};
    /** Result */
    PointSet result_;
    /** Computation status */
    Status status_{Status::Success};
};
}  // namespace volcart::segmentation
