#pragma once

#include <opencv2/core.hpp>

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/Mixins.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart
{
namespace texturing
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
 * @see volcart::PerPixelMap
 * @ingroup Texture
 */
class PPMGenerator : public IterationsProgress
{
public:
    /**@{*/
    enum class Shading { Flat = 0, Smooth };

    /** Default constructor */
    PPMGenerator() : width_{0}, height_{0} {}

    /** Construct with dimension parameters */
    PPMGenerator(size_t h, size_t w) : width_{w}, height_{h} {}
    /**@}*/

    /**@{*/
    /** @brief Set the input mesh */
    void setMesh(const ITKMesh::Pointer& m) { inputMesh_ = m; }

    /** @brief Set the input UV map */
    void setUVMap(const UVMap& u) { uvMap_ = u; }
    /**@}*/

    /**@{*/
    /** @brief Set the dimensions of the output PPM */
    void setDimensions(size_t h, size_t w);

    /** @brief Set the normal shading method */
    void setShading(Shading s) { shading_ = s; }
    /**@}*/

    /**@{*/
    /** @brief Compute the PerPixelMap */
    PerPixelMap compute();
    /**@}*/

    /**@{*/
    /** @brief Get the generated PerPixelMap */
    const PerPixelMap getPPM() const { return ppm_; }

    /** @copydoc getPPM() const */
    PerPixelMap getPPM() { return ppm_; }

    /**
     * @brief Get the generated cell map
     *
     * The cell map is an image containing the face index associated with each
     * pixel in the PPM. The cell map is of type `CV_32SC1`. Pixels without
     * a face assignment have value equal to -1.
     */
    cv::Mat getCellMap() const { return cellMap_; }
    /**@}*/

    /** @brief Returns the maximum progress value */
    size_t progressIterations() const override { return width_ * height_; }

private:
    /** Triangular face with 2D-to-3D correspondence */
    struct CellInfo {
        /** Clear the storage vectors */
        void reset()
        {
            pts2D.clear();
            pts3D.clear();
            normals.clear();
        }
        /** 2D vertices */
        std::vector<cv::Vec3d> pts2D;
        /** 3D vertices */
        std::vector<cv::Vec3d> pts3D;
        /** 3D surface normal */
        std::vector<cv::Vec3d> normals;
    };

    /** Preprocess mesh to aid in correspondence look ups */
    void generate_centroid_mesh_();
    /** Generate the PerPixelMap */
    void generate_ppm_();
    /** Find the cell which belongs to a pixel */
    void find_cell_(size_t x, size_t y, size_t& cellHint);
    /** Convert from Barycentric coordinates to an interpolated normal */
    static cv::Vec3d GouraudNormal(
        const cv::Vec3d& nUVW,
        const cv::Vec3d& nA,
        const cv::Vec3d& nB,
        const cv::Vec3d& nC);

    /** Input mesh */
    ITKMesh::Pointer inputMesh_;
    /** Input UV Map */
    UVMap uvMap_;
    /** Mesh where each vertex is the centroid of a face from the 2D mesh */
    ITKMesh::Pointer centroidMesh_;
    /** Face correspondence information */
    std::vector<CellInfo> cellInformation_;
    /** kdTree for cell lookups */
    ITKPointsLocator::Pointer kdTree_;
    /** kdTree search size */
    size_t kdSearchSize_{100};

    /** Working mesh */
    ITKMesh::Pointer workingMesh_;
    /** Output PerPixelMap */
    PerPixelMap ppm_;
    /** Output PPM mask */
    cv::Mat mask_;
    /** Output cell map */
    cv::Mat cellMap_;
    /** Output shading */
    Shading shading_{Shading::Smooth};
    /** Output width of the PerPixelMap */
    size_t width_;
    /** Output height of the PerPixelMap */
    size_t height_;
};
}  // namespace texturing
}  // namespace volcart
