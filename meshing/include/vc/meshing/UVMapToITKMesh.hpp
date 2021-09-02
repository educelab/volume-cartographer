#pragma once

/** @file */

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart::meshing
{

/**
 * @brief Convert a UVMap to a mesh
 *
 * Combines the UV coordinates from a UVMap and the face information from an
 * ITKMesh to produce a meshed UVMap.
 */
class UVMapToITKMesh
{
public:
    /** @brief Set the input mesh */
    void setMesh(const ITKMesh::Pointer& m);
    /** @brief Set the input UV Map */
    void setUVMap(UVMap::Pointer u);

    /**
     * @brief If true, scale the output mesh to the width/height stored in the
     * UVMap. Default: false
     */
    void setScaleToUVDimensions(bool b);

    /** @brief Compute the UV mesh */
    auto compute() -> ITKMesh::Pointer;

    /** @brief Get the computed UV mesh */
    [[nodiscard]] auto getUVMesh() const -> ITKMesh::Pointer;

private:
    /** Input mesh */
    ITKMesh::Pointer inputMesh_;
    /** UV map */
    UVMap::Pointer uvMap_;
    /** Do mesh scaling */
    bool scaleMesh_{false};
    /** Output mesh */
    ITKMesh::Pointer outputMesh_;
};

}  // namespace volcart::meshing