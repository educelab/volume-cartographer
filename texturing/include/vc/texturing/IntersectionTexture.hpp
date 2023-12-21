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
    static auto New() -> Pointer;

    /** Default constructor */
    IntersectionTexture() = default;
    /** Default destructor */
    ~IntersectionTexture() override = default;
    /** Default copy constructor */
    IntersectionTexture(IntersectionTexture&) = default;
    /** Default move constructor */
    IntersectionTexture(IntersectionTexture&&) = default;
    /** Default copy operator */
    auto operator=(const IntersectionTexture&)
        -> IntersectionTexture& = default;
    /** Default move operator */
    auto operator=(IntersectionTexture&&) -> IntersectionTexture& = default;

    /**@{*/
    /** @brief Compute the Texture */
    auto compute() -> Texture override;
    /**@}*/
};
}  // namespace volcart::texturing
