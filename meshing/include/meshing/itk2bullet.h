//
// Created by Abigail Coleman 10/21/15
//
#pragma once

#ifdef VC_USE_BULLET

#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>

#include "common/vc_defines.h"

namespace volcart
{
namespace meshing
{

class itk2bullet
{
public:
    itk2bullet(
        ITKMesh::Pointer input,
        btSoftBodyWorldInfo& worldInfo,
        btSoftBody** output);
};

class bullet2itk
{
public:
    bullet2itk(btSoftBody* softBody, ITKMesh::Pointer output);
};

}  // namespace meshing
}  // namespace volcart

#endif  // VC_USE_BULLET
