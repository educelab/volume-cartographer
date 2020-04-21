#pragma once

#include "vc/core/types/BoundingBox.hpp"
#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart
{
namespace segmentation
{
/**
 * @class ChainSegmentationAlgorithmBaseClass
 * @author Seth Parker
 * @brief Base class for segmentation algorithms that propagate a collected
 * chain of points
 */
class ChainSegmentationAlgorithmBaseClass
{
public:
    /** Default destructor for virtual base class */
    virtual ~ChainSegmentationAlgorithmBaseClass() = default;

    /** Chain type */
    using Chain = std::vector<cv::Vec3d>;
    /** PointSet type */
    using PointSet = volcart::OrderedPointSet<cv::Vec3d>;
    /** Bounding Box type */
    using Bounds = volcart::BoundingBox<double, 3>;
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
    void setNumberOfSteps(size_t n) { numSteps_ = n; }

    /** @brief Set the propagation step size
     *
     * Typically the expected unit distance between each output iteration in
     * voxel units.
     */
    void setStepSize(double s) { stepSize_ = s; }
    /**@}*/

    /**@{*/
    /** @brief Compute the Texture */
    virtual PointSet compute() = 0;

    /** @brief Get the status of the previous computation */
    Status getStatus() const { return status_; }

    /** @brief Get progress as percent of total */
    virtual float getProgress() const = 0;
    /**@}*/

    /**@{*/
    /** @brief Get the generated Texture */
    const PointSet& getPointSet() const { return result_; }

    /** @copydoc getTexture() const */
    PointSet& getPointSet() { return result_; }
    /**@}*/

protected:
    /** Default constructor */
    ChainSegmentationAlgorithmBaseClass() = default;
    /** Volume */
    Volume::Pointer vol_;
    /** Seed chain */
    Chain startingChain_;
    /** Bounding box */
    Bounds bb_;
    /** Number of propagation steps */
    size_t numSteps_{0};
    /** Propagation step size */
    double stepSize_{1.0};
    /** Result */
    PointSet result_;
    /** Computation status */
    Status status_{Status::Success};
    /** progress */
    float progress_{0.0};
};
}
}
