#pragma once

#include <opencv2/core.hpp>

#include "vc/core/types/Texture.hpp"
#include "vc/core/types/UVMap.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/vc_defines.hpp"
#include "vc/texturing/TexturingUtils.hpp"

namespace volcart
{
namespace texturing
{
/**
 * @class CompositeTextureV2
 * @author Seth Parker
 * @date 12/18/15
 *
 * @brief Generate a Texture using a variety of composite volume filters
 *
 * This class generates a texture image by filtering the Volume using one of a
 * number of composite measures:
 *
 * - Intersection: Select the intensity value that lies at the intersection
 * point between the mesh and volume.
 * - Non-Max Suppression: Filter a neighborhood to select the maximum intensity.
 * Suppress voxels that are not the local maxima. For linear neighborhoods, this
 * is the same as the Maximum filter.
 * - Maximum: Filter a neighborhood to select the maximum intensity.
 * - Minimum: Filter a neighborhood to select the minimum intensity.
 * - Median + Averaging: Filter a neighborhood by averaging the median 50%.
 * - Median: Filter a neighborhood to select the median intensity.
 * - Mean: Filter a neighborhood by averaging the intensities.
 *
 * @ingroup Texture
 */
class CompositeTextureV2
{
public:
    /**@{*/
    /**
     * @brief Construct with input parameters
     *
     * Texturing process is run on construction.
     *
     * @param input Input mesh
     * @param volpkg Input volume package
     * @param uvMap Input UV map for the mesh
     * @param radius Radius of the filtering neighborhood
     * @param width Width of the output image
     * @param height Height of the output image
     * @param method Filtering method
     * @param direction Filtering direction
     */
    CompositeTextureV2(
        ITKMesh::Pointer input,
        VolumePkg& volpkg,
        UVMap uvMap,
        double radius,
        size_t width,
        size_t height,
        CompositeOption method = CompositeOption::NonMaximumSuppression,
        DirectionOption direction = DirectionOption::Bidirectional);
    /**@}*/

    /**@{*/
    /** @brief Get the generated Texture */
    const Texture& getTexture() const { return texture_; }

    /** @copydoc getTexture() const */
    Texture& getTexture() { return texture_; }
    /**@}*/

private:
    /** Generate the Texture */
    int process_();

    /** Input mesh */
    ITKMesh::Pointer input_;
    /** Input UV map */
    UVMap uvMap_;
    /** Output Texture */
    Texture texture_;

    /** Input VolumePkg */
    VolumePkg& volpkg_;
    /** Width of the output image */
    size_t width_;
    /** Height of the output image */
    size_t height_;
    /** Radius of the filtering neighborhood */
    double radius_;
    /** Filtering method */
    CompositeOption method_;
    /** Filtering direction */
    DirectionOption direction_;
};
}
}
