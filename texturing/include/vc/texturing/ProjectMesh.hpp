#pragma once

/** @file */

#include <itkCompositeTransform.h>

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/PerPixelMap.hpp"

namespace volcart::texturing
{
/**
 * @brief Generates a PerPixelMap by projecting a mesh onto an image plane
 *
 * Constructs an oriented bounding box around a mesh, chooses the largest plane
 * as the image plane, and iterates over the surface of this plane. At each
 * point, projects a ray oriented with the bounding box into the mesh. The
 * position and surface normal at the mesh intersection point are stored in the
 * PPM.
 *
 * The input mesh should separately be modified using rt_reorder_texture from
 * registration-toolkit if the PPM should align with the texture image.
 * Optionally, if the mesh already has a reordered texture and a separate image
 * has been registered to that texture image, the separate target image and its
 * registration transform can be passed. In this scenario this class will
 * iterate over the provided target image space, first mapping each point using
 * the transform to find its corresponding location in the mesh bounding box
 * plane before performing the ray intersection. The inverse transform may be
 * specified if registration was performed with the mesh texture as the fixed
 * image.
 *
 * By default, the last mesh intersection point is used for each ray, optionally
 * this may be changed to the first intersection point.
 *
 * This class uses raytracing functionality provided by the
 * [bvh library](https://github.com/madmann91/bvh).
 *
 * @see volcart::PerPixelMap
 * @ingroup Texture
 */
class ProjectMesh
{
public:
    /**@{*/
    /** @brief Target image space sampling method */
    enum class SampleMode {
        /**
         * @brief Samples per unit interval in mesh space determine PPM
         * dimensions
         */
        Rate,
        /** @brief PPM dimensions determine image plane sample rate */
        Dimensions
    };
    using CompositeTransform = itk::CompositeTransform<double, 2>;
    using Point = itk::Point<double, 2>;

    /** @brief Default constructor */
    ProjectMesh() = default;
    /**@}*/

    /**@{*/
    /** @brief Set the input mesh */
    void setMesh(const ITKMesh::Pointer& inputMesh);
    /**@}*/

    /**@{*/
    /** @brief Set the image transform from mesh texture to target */
    void setTransform(CompositeTransform::Pointer transform);
    /** @brief Invert the image transform prior to use */
    void setUseInverseTransform(bool useInverse);
    /** @brief Set the PPM space sampling mode */
    void setSampleMode(SampleMode mode);
    /** @brief Set the texture image dimensions */
    void setTextureDimensions(int width, int height);
    /** @brief Set the PPM dimensions */
    void setPPMDimensions(int width, int height);
    /** @brief Set the X and Y sample rate for the PPM */
    void setSampleRate(double rate);
    /**@}*/

    /**@{*/
    /** @brief Use the first mesh intersection rather than the last */
    void setUseFirstIntersection(bool useFirstIntersection);
    /**@}*/

    /**@{*/
    /** @brief Project mesh and compute PerPixelMap with given settings */
    auto compute() -> PerPixelMap;
    /**@}*/

private:
    /** Input mesh */
    ITKMesh::Pointer inputMesh_;
    /** Output PPM */
    PerPixelMap outputPPM_;

    /** Transform from target image to PPM space */
    CompositeTransform::Pointer tfm_;
    /** Invert transform prior to use */
    bool useInverse_{false};

    /** PPM sampling mode */
    SampleMode mode_{SampleMode::Rate};
    /** Texture image width in pixels */
    int textureWidth_{1};
    /** Texture image height in pixels */
    int textureHeight_{1};
    /** PerPixelMap width in pixels */
    int ppmWidth_{0};
    /** PerPixelMap height in pixels */
    int ppmHeight_{0};
    /** PerPixelMap sample rate in X axis */
    double sampleRateX_{1.0};
    /** PerPixelMap sample rate in Y axis */
    double sampleRateY_{1.0};

    /** Use the first mesh intersection rather than the last */
    bool useFirstIntersection_;
};
}  // namespace volcart::texturing
