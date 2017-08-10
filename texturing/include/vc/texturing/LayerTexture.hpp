#pragma once

#include "vc/texturing/TexturingAlgorithmBaseClass.hpp"

namespace volcart
{
namespace texturing
{
/**
 * @class LayerTexture
 * @author Seth Parker
 * @date 11/24/2016
 *
 * @brief Generate a Texture of layered images
 *
 * The Texture generated by this class contains multiple texture images. Each
 * image is the projection of the mesh some distance through the Volume along
 * each point's surface normal. For well-formed meshes and parameterizations,
 * this amounts to resampling the Volume into a flattened subvolume with the
 * segmentation mesh forming a straight line at its center.
 *
 * @ingroup Texture
 */
class LayerTexture : public TexturingAlgorithmBaseClass
{
public:
    /**@{*/
    /** @brief Compute the Texture */
    Texture compute() override;
    /**@}*/
};

}  // texturing
}  // volcart