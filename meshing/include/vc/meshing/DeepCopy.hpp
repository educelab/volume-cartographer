#pragma once

/** @file */

#include "vc/core/types/ITKMesh.hpp"

namespace volcart
{
namespace meshing
{
/**
 * @brief Create exact copy of ITKMesh.
 *
 * Copy vertex and face information from the input mesh into the output mesh.
 * The resulting mesh is a unique (i.e. memory independent) copy of the
 * original.
 *
 * @warning Output parameter should point to an empty ITKMesh. This function
 * does not initialize a new ITKMesh::Pointer.
 *
 * @param copyVertices If `true,` copy vertices into the target mesh. Default:
 * `true`
 * @param copyFaces If `true,` copy faces into the target mesh. Default: `true`
 *
 * @ingroup Meshing
 */
void DeepCopy(
    const ITKMesh::Pointer& input,
    const ITKMesh::Pointer& output,
    bool copyVertices = true,
    bool copyFaces = true);
}  // namespace meshing
}  // namespace volcart
