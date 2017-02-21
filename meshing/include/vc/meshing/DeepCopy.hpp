#pragma once

#include "vc/core/vc_defines.hpp"

namespace volcart
{
namespace meshing
{
/**
 * @class DeepCopy
 * @author Seth Parker
 * @date 12/21/15
 *
 * @brief Create exact copy of ITKMesh.
 *
 * Copy all vertex and face information from the input mesh into the output
 * mesh. The resulting mesh is a unique (i.e. memory independent) copy of the
 * original.
 *
 * @warning Output parameter should point to an empty ITKMesh. This function
 * will not initialize a new ITKMesh::Pointer.
 *
 * @ingroup Meshing
 */
class DeepCopy
{
public:
    DeepCopy(ITKMesh::Pointer input, ITKMesh::Pointer output);
};
}
}
