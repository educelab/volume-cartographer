#pragma once

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart
{
namespace texturing
{

class FlatteningAlgorithmBaseClass
{
public:
    /** Default destructor for virtual base class */
    virtual ~FlatteningAlgorithmBaseClass() = default;

    /**@{*/
    /** @brief Set the input Mesh */
    void setMesh(ITKMesh::Pointer mesh) { mesh_ = std::move(mesh); }
    /**@}*/

    /**@{*/
    /** @brief Compute the parameterization */
    virtual ITKMesh::Pointer compute() = 0;
    /**@}*/

    /**@{*/
    /** @brief Get the generated Texture */
    UVMap getUVMap();
    /**@}*/

protected:
    ITKMesh::Pointer mesh_;
    ITKMesh::Pointer output_;
};
}
}
