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
    void setUVMap(UVMap u);

    /**
     * @brief If true, scale the output mesh to the width/height stored in the
     * UVMap. Default: false
     */
    void setScaleToUVDimensions(bool b);

    /** @brief Compute the UV mesh */
    ITKMesh::Pointer compute();
    /** @brief Get the computed UV mesh */
    ITKMesh::Pointer getUVMesh() const;

private:
    /** Input mesh */
    ITKMesh::Pointer inputMesh_;
    /** UV map */
    UVMap inputUVMap_;
    /** Do mesh scaling */
    bool scaleMesh_{false};
    /** Output mesh */
    ITKMesh::Pointer outputMesh_;
};

}  // namespace volcart::meshing