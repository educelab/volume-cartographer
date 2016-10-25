
#pragma once

#ifdef VC_USE_BULLET

#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>

#include "core/vc_defines.h"

namespace volcart
{
namespace meshing
{
/**
 * @class itk2bullet
 * @author Abigail Coleman
 * @date 10/21/15
 *
 * @brief Converts between ITK Mesh and a Bullet Soft body
 *
 * This class copies vertex information between the two and
 * then converts either cells of the mesh into faces
 * @newline
 *
 * This class can only be used if Bullet is found.
 *
 * @newline
 * This is used for performing virtual unraveling.
 *
 * @ingroup Meshing
 *
 */
class itk2bullet
{
public:
    /**
    * @brief Converts an itk Mesh into a Bullet softbody
    *
    * This functions takes in an ITK mesh then copies the
    * points to a Bullet soft body. It then converts the
    * cells into faces.
    *
    * @param input Mesh that needs to be converted
    * @param worldInfo Gives information about the container
    *        for the soft body
    * @param output Bullet Soft body that models the mesh
    */
    itk2bullet(
        ITKMesh::Pointer input,
        btSoftBodyWorldInfo& worldInfo,
        btSoftBody** output);
};

/**
 * @class bullet2itk
 * @author Abigail Coleman
 * @date 10/21/15
 *
 * @brief Converts between ITK Mesh and a Bullet Soft body
 *
 * This class copies vertex information between the two and
 * then converts the faces into cells .
 * @newline
 *
 * This class can only be used if Bullet is found.
 *
 * @newline
 * This is used for performing virtual unraveling.
 *
 * @ingroup Meshing
 *
 */

class bullet2itk
{
public:
    /**
    * @brief Converts a Bullet softbody into an itk Mesh
    *
    * This class takes in a bullet soft body and
    * copies the vertex information. It then converts
    * the faces into cells.
    *
    * @param softbody body that needs to be converted
    * @param output Pointer to the mesh that was created
    */
    bullet2itk(btSoftBody* softBody, ITKMesh::Pointer output);
};

}  // namespace meshing
}  // namespace volcart

#endif  // VC_USE_BULLET
