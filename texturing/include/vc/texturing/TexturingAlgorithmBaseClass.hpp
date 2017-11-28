#pragma once

#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/Texture.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart
{
namespace texturing
{
class TexturingAlgorithmBaseClass
{
public:
    /** Default destructor for virtual base class */
    virtual ~TexturingAlgorithmBaseClass() = default;

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
    /**@}*/

    /**@{*/
    /** @brief Compute the Texture */
    virtual Texture compute() = 0;
    /**@}*/

    /**@{*/
    /** @brief Get the generated Texture */
    const Texture& getTexture() const { return result_; }

    /** @copydoc getTexture() const */
    Texture& getTexture() { return result_; }
    /**@}*/

protected:
    /** Default constructor */
    TexturingAlgorithmBaseClass() = default;

    /** PPM */
    PerPixelMap ppm_;
    /** Volume */
    Volume::Pointer vol_;
    /** Search radius */
    double radius_{1.0};
    /** Search direction */
    Direction direction_{Direction::Bidirectional};
    /** Search sampling interval */
    double interval_{1.0};
    /** Result */
    Texture result_;

    /** Calculate the size of each neighborhood based on current options */
    size_t neighborhood_count_();
};
}
}
