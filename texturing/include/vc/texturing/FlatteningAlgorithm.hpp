#pragma once

/** @file */

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart::texturing
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
    UVMap::Pointer getUVMap();
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
};
}  // namespace volcart::texturing
