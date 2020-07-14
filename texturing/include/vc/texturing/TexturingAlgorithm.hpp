#pragma once

#include <memory>

#include "vc/core/neighborhood/NeighborhoodGenerator.hpp"
#include "vc/core/types/Mixins.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/Texture.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart
{
namespace texturing
{
class TexturingAlgorithm : public IterationsProgress
{
public:
    /** Pointer type */
    using Pointer = std::shared_ptr<TexturingAlgorithm>;

    /** Default destructor for virtual base class */
    virtual ~TexturingAlgorithm() = default;

    /**@{*/
    /** @brief Set the input PerPixelMap */
    void setPerPixelMap(PerPixelMap ppm) { ppm_ = std::move(ppm); }

    /** @brief Set the input Volume */
    void setVolume(Volume::Pointer vol) { vol_ = std::move(vol); }
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

    /** @brief Returns the maximum progress value */
    size_t progressIterations() const override
    {
        return ppm_.getMappings().size();
    }

protected:
    /** Default constructor */
    TexturingAlgorithm() = default;

    /** PPM */
    PerPixelMap ppm_;
    /** Volume */
    Volume::Pointer vol_;

    /** Result */
    Texture result_;
};
}  // namespace texturing
}  // namespace volcart
