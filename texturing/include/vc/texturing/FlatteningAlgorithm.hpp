#pragma once

/** @file */

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart
{
namespace texturing
{

class FlatteningAlgorithm
{
public:
    /** Shared pointer type */
    using Pointer = std::shared_ptr<FlatteningAlgorithm>;

    /** Default destructor for virtual base class */
    virtual ~FlatteningAlgorithm() = default;

    /**@{*/
    /** @brief Set the input Mesh */
    void setMesh(const ITKMesh::Pointer& m) { mesh_ = m; }
    /**@}*/

    /**@{*/
    /** @brief Compute the parameterization */
    virtual ITKMesh::Pointer compute() = 0;
    /**@}*/

    /**@{*/
    /** @brief Get the parameterized mesh */
    ITKMesh::Pointer getMesh() { return output_; }

    /** @brief Get the parameterized mesh as a UVMap  */
    UVMap getUVMap();
    /**@}*/

protected:
    /** Default constructor */
    FlatteningAlgorithm() = default;

    /** Constructor with input mesh initialization */
    explicit FlatteningAlgorithm(const ITKMesh::Pointer& m) { mesh_ = m; }

    /** Input mesh */
    ITKMesh::Pointer mesh_;
    /** Output mesh */
    ITKMesh::Pointer output_;

    /**
     * Reorient the UV map such that the Z-axis of the input mesh is parallel to
     * the V-axis of the UV map. Assumes that the UV coordinates are stored in
     * {output_.pt[0] = u, output_.pt[2] = v}.
     */
    void orient_uvs_();
};
}  // namespace texturing
}  // namespace volcart
