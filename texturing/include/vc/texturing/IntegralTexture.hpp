#pragma once

#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/Texture.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart
{
namespace texturing
{

class IntegralTexture
{
public:
    enum class Weight { Positive, Negative, None };

    /**@{*/
    /** @brief Set the input PerPixelMap */
    void setPerPixelMap(PerPixelMap ppm) { ppm_ = std::move(ppm); }

    /** @brief Set the input Volume */
    void setVolume(Volume::Pointer vol) { vol_ = std::move(vol); }

    /** @brief Set the sampling search radius: the distance from the mesh to
     * consider for compositing */
    void setSamplingRadius(double r) { radius_ = r; }

    /**
     * @brief Set the sampling interval: how frequently the voxels along the
     * radius are sampled for compositing purposes
     *
     * Default = 1.0
     */
    void setSamplingInterval(double i) { interval_ = i; }

    /**
     * @brief Set the filtering search direction: which "side" of the mesh to
     * consider when compositing
     *
     * Default: Bidirectional
     */
    void setSamplingDirection(Direction d) { direction_ = d; }

    void setWeight(Weight w) { weightType_ = w; }
    /**@}*/

    /**@{*/
    /** @brief Compute the Texture */
    Texture compute();
    /**@}*/

    /**@{*/
    /** @brief Get the generated Texture */
    const Texture& getTexture() const { return result_; }

    /** @copydoc getTexture() const */
    Texture& getTexture() { return result_; }
    /**@}*/

private:
    Volume::Pointer vol_;
    PerPixelMap ppm_;
    double radius_;
    double interval_{1.0};
    Direction direction_{Direction::Bidirectional};

    Texture result_;

    Weight weightType_{Weight::None};
    double currentWeight_;
    double weightStep_;
    void setup_weights_(size_t s);
};

}  // texturing
}  // volcart
