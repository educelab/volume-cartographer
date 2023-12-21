#pragma once

/** @file */

#include <opencv2/core.hpp>

#include "vc/core/types/ITKMesh.hpp"

namespace volcart::meshing
{

/**
 * @class OrientNormals
 *
 * @brief Orient vertex normals towards a reference point.
 *
 * If vertex normals are flipped, the face normals are also flipped by
 * reversing the winding order of all faces. This assumes that vertex normals
 * were originally derived from the face normals and point in generally the
 * same direction as the face normals.
 */
class OrientNormals
{
public:
    /** @brief Reference point mode */
    enum class ReferenceMode {
        Centroid,  ///< Mesh centroid reference point for convex meshes.
        Manual     ///< Point provided by setReferencePoint().
    };

    /** @brief Default constructor */
    OrientNormals() = default;

    /** @brief Set the input mesh */
    void setMesh(const ITKMesh::Pointer& mesh);

    /** @brief Get the output mesh with updated normals */
    [[nodiscard]] auto getMesh() const -> ITKMesh::Pointer;

    /**
     * @brief Set reference point mode
     *
     * Determines which automatically calculated reference point to use. See
     * ReferenceMode for more details.
     */
    void setReferenceMode(ReferenceMode mode);

    /** @brief Get the current reference mode */
    [[nodiscard]] auto referenceMode() const -> ReferenceMode;

    /**
     * @brief Set manual reference point
     *
     * If provided, automatically sets the reference mode to
     * ReferenceMode::Manual.
     */
    void setReferencePoint(const cv::Vec3d& point);

    /** @brief Get the manually defined reference point */
    [[nodiscard]] auto referencePoint() const -> cv::Vec3d;

    /** @brief Compute vertex normal reorientation */
    auto compute() -> ITKMesh::Pointer;

private:
    /** Input mesh */
    ITKMesh::Pointer input_;
    /** Reference point mode */
    ReferenceMode mode_{ReferenceMode::Centroid};
    /** User-defined reference point */
    cv::Vec3d refPt_;
    /** Output mesh */
    ITKMesh::Pointer output_;
};
}  // namespace volcart::meshing
