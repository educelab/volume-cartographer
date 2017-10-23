#pragma once

#include "vc/texturing/TexturingAlgorithmBaseClass.hpp"

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
class IntersectionTexture : public TexturingAlgorithmBaseClass
{
public:
    /** Default destructor */
    ~IntersectionTexture() = default;

    /**@{*/
    /** @brief Compute the Texture */
    Texture compute() override;
    /**@}*/
};
}
}
