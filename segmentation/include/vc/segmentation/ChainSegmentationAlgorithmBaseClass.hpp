#pragma once

#include "vc/core/types/BoundingBox.hpp"
#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart
{
namespace segmentation
{
class ChainSegmentationAlgorithmBaseClass
{
public:
    using Chain = std::vector<cv::Vec3d>;
    using PointSet = volcart::OrderedPointSet<cv::Vec3d>;
    using Bounds = volcart::BoundingBox<double, 3>;

    enum class Status { Success, Failure, ReturnedEarly };

    /**@{*/
    /** @brief Set the input Volume */
    void setVolume(Volume::Pointer vol)
    {
        vol_ = std::move(vol);
        bb_ = vol_->bounds();
    }

    void setBounds(Bounds b) { bb_ = std::move(b); }

    void setChain(Chain c) { chain_ = std::move(c); }

    void setStepSize(double s) { stepSize_ = s; }
    /**@}*/

    /**@{*/
    /** @brief Compute the Texture */
    virtual PointSet compute() = 0;

    Status getStatus() const { return status_; }
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
    Chain chain_;
    Bounds bb_;
    double stepSize_{1.0};
    /** Result */
    PointSet result_;
    Status status_{Status::Success};
};
}
}
