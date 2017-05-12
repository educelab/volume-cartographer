#pragma once

#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/Texture.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart
{
namespace texturing
{

/**
 * @class IntersectionTexture
 * @author Seth Parker
 * @date 05/15/2017
 *
 * @brief Generate a Texture by intersection with a Volume
 *
 * @ingroup Texture
 */
class IntersectionTexture
{
public:
    /**@{*/
    /** @brief Set the input PerPixelMap */
    void setPerPixelMap(PerPixelMap ppm) { ppm_ = std::move(ppm); }

    /** @brief Set the input Volume */
    void setVolume(Volume::Pointer vol) { vol_ = std::move(vol); }
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
    /** PPM */
    PerPixelMap ppm_;
    /** Volume */
    Volume::Pointer vol_;
    /** Output */
    Texture result_;
};
}
}
