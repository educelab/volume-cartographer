#pragma once

/** @file */

#include "vc/texturing/TexturingAlgorithm.hpp"

namespace volcart::texturing
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
class IntersectionTexture : public TexturingAlgorithm
{
public:
    /** Pointer type */
    using Pointer = std::shared_ptr<IntersectionTexture>;

    /** Make shared pointer */
    static Pointer New() { return std::make_shared<IntersectionTexture>(); }

    /** Default destructor */
    ~IntersectionTexture() override = default;

    /**@{*/
    /** @brief Compute the Texture */
    Texture compute() override;
    /**@}*/
};
}  // namespace volcart::texturing
