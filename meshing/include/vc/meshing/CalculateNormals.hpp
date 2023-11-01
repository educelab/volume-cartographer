#pragma once

/** @file */

#include <iostream>

#include <opencv2/core.hpp>

#include "vc/core/types/ITKMesh.hpp"
#include "vc/meshing/DeepCopy.hpp"

namespace volcart::meshing
{
/**
 * @class CalculateNormals
 * @author Hannah Hatch
 * @date 7/26/16
 *
 * @brief Calculate vertex normals for ITK Meshes.
 *
 * Given an ITK mesh, generates a copy of that mesh with embedded vertex
 * normals.
 *
 * @ingroup Meshing
 */
class CalculateNormals
{
public:
    //** @name Constructors */
    //@{
    CalculateNormals() = default;

    /**
     * @param mesh Input Mesh whose normals you want computed
     */
    explicit CalculateNormals(const ITKMesh::Pointer& mesh)
        : input_{mesh}, output_{ITKMesh::New()}
    {
        DeepCopy(input_, output_);
    }
    //@}

    //** @name Input/Output */
    //@{
    /**
     * @brief Set the input mesh.
     * @param mesh The mesh for which normals will be calculated
     */
    void setMesh(const ITKMesh::Pointer& mesh);

    /**
     * @brief Get the output mesh with computed normals
     */
    ITKMesh::Pointer getMesh() const { return output_; }
    //@}

    /**
     * @brief Compute vertex normals for the mesh.
     */
    ITKMesh::Pointer compute();

    /**
     * @brief Flag specifying if we should reorient normals.
     * 
     * If this flag is set and most of the mesh's normals are facing outwards
     * relative to the mesh's centroid, then they're considered flipped, and
     * will be reversed to face inwards, the conventionally positive direction.
     */
    bool shouldOrientNormals = true;

private:
    /**
     * @brief Compute normals for each vertex.
     *
     * For each face, computes the normal to that face and adds the resulting
     * vector to a sum vector for each vertex in that face.
     */
    void compute_normals_();

    /**
     * @brief Assign the summed normals to the output mesh.
     *
     * Takes the summed normals for each vertex and assigns them to the
     * corresponding vertex in the output mesh.
     */
    void assign_to_mesh_();

    /**
     * @brief Compute the mesh's centroid, the arithmetic mean of its vertices.
     */
    cv::Vec3d compute_mesh_centroid_();

    /** Mesh for which normals will be calculated. */
    ITKMesh::Pointer input_;

    /** Mesh with calculated normals. */
    ITKMesh::Pointer output_;

    /** Storage for summed normals, organized by vertex ID */
    std::vector<cv::Vec3d> vertexNormals_;

    bool flippedNormals_;
};
}  // namespace volcart::meshing
