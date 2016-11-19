#pragma once

#include "core/vc_defines.h"

namespace volcart
{
namespace meshing
{
/**
 * @class deepCopy
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
class deepCopy
{
public:
    deepCopy(ITKMesh::Pointer input, ITKMesh::Pointer output);
};  // deepCopy
}  // meshing
}  // volcart
