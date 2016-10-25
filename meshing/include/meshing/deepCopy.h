
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
 * @ingroup Meshing
 */
class deepCopy
{
public:
    /**
     * @brief Copies one mesh to another
     *
     * This function takes in two pointers to ITK meshes and then
     * copies the input mesh onto the output mesh. This is done
     * by copying all of the points and then all of the faces.
     *
     * @param input Mesh that should be copied
     * @param output Where you want the copy to be stored
     */
    deepCopy(ITKMesh::Pointer input, ITKMesh::Pointer output);
};  // deepCopy
}  // meshing
}  // volcart
