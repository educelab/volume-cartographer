#pragma once

/** @file */

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/Mixins.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart::texturing
{
/**
 * @class PPMGenerator
 * @brief Generates a PerPixelMap from an ITKMesh and a UVMap
 *
 * Rasters a UVMap and embeds the raster with 2D-to-3D lookup information. Each
 * pixel in the PPM stores a vector of six double-precision floats which
 * correspond to the 3D position and normal vector associated with that pixel:
 * `{x, y, z, nx, ny, nz}`
 *
 * This class uses raytracing functionality provided by the
 * [bvh library](https://github.com/madmann91/bvh).
 *
 * @see volcart::PerPixelMap
 * @ingroup Texture
 */
class PPMGenerator : public IterationsProgress
{
public:
    /**@{*/
    enum class Shading { Flat = 0, Smooth };

    /** Default constructor */
    PPMGenerator() = default;

    /** Construct with dimension parameters */
    PPMGenerator(size_t h, size_t w);
    /**@}*/

    /**@{*/
    /** @brief Set the input mesh */
    void setMesh(const ITKMesh::Pointer& m);

    /** @brief Set the input UV map */
    void setUVMap(const UVMap& u);
    /**@}*/

    /**@{*/
    /** @brief Set the dimensions of the output PPM */
    void setDimensions(size_t h, size_t w);

    /** @brief Set the normal shading method */
    void setShading(Shading s);
    /**@}*/

    /**@{*/
    /** @brief Compute the PerPixelMap */
    PerPixelMap compute();
    /**@}*/

    /**@{*/
    /** @brief Get the generated PerPixelMap */
    PerPixelMap getPPM() const;

    /** @brief Returns the maximum progress value */
    size_t progressIterations() const override;

private:

    /** Input mesh */
    ITKMesh::Pointer inputMesh_;
    /** Input UV Map */
    UVMap uvMap_;

    /** Working mesh */
    ITKMesh::Pointer workingMesh_;
    /** Output PerPixelMap */
    PerPixelMap ppm_;
    /** Output shading */
    Shading shading_{Shading::Smooth};
    /** Output width of the PerPixelMap */
    size_t width_{0};
    /** Output height of the PerPixelMap */
    size_t height_{0};
};
}  // namespace volcart::texturing
