#pragma once

#include "vc/core/types/ITKMesh.hpp"

namespace volcart::meshing
{
class OrientNormals
{
public:
    OrientNormals() = default;

    /**
     * @brief Set the input mesh.
     * @param mesh The mesh for which normals will be calculated
     */
    void setMesh(const ITKMesh::Pointer& mesh);

    /**
     * @brief Get the output mesh with computed normals
     */
    [[nodiscard]] auto getMesh() const -> ITKMesh::Pointer;

    /**
     * @brief Compute vertex normals for the mesh.
     */
    auto compute() -> ITKMesh::Pointer;

private:
    ITKMesh::Pointer input_;
    ITKMesh::Pointer output_;
};
}  // namespace volcart::meshing
