#pragma once

/** @file */

#include <cstddef>
#include <memory>

#include "vc/core/neighborhood/NeighborhoodGenerator.hpp"
#include "vc/core/types/Mixins.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart::texturing
{
class TexturingAlgorithm : public IterationsProgress
{
public:
    /** Pointer type */
    using Pointer = std::shared_ptr<TexturingAlgorithm>;

    /** Image outputs */
    using Texture = std::vector<cv::Mat>;

    /** Default destructor for virtual base class */
    virtual ~TexturingAlgorithm() = default;

    /** @brief Set the input PerPixelMap */
    void setPerPixelMap(PerPixelMap::Pointer ppm) { ppm_ = std::move(ppm); }

    /** @brief Set the input Volume */
    void setVolume(Volume::Pointer vol) { vol_ = std::move(vol); }

    /** @brief Compute the Texture */
    virtual auto compute() -> Texture = 0;

    /** @brief Get the generated Texture */
    auto getTexture() -> Texture { return result_; }

    /** @brief Returns the maximum progress value */
    [[nodiscard]] auto progressIterations() const -> std::size_t override
    {
        return ppm_->getMappings().size();
    }

protected:
    /** Default constructor */
    TexturingAlgorithm() = default;

    /** PPM */
    PerPixelMap::Pointer ppm_;
    /** Volume */
    Volume::Pointer vol_;

    /** Result */
    Texture result_;
};
}  // namespace volcart::texturing
