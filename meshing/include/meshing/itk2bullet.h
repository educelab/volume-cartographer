//
// Created by Abigail Coleman 10/21/15
//
#pragma once

#ifdef VC_USE_BULLET

#include "common/vc_defines.h"
#include <LinearMath/btScalar.h>
#include <LinearMath/btVector3.h>

// Bullet Soft Body
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>

namespace volcart {
  namespace meshing {

    class itk2bullet {
    public:
        itk2bullet( VC_MeshType::Pointer input, btSoftBodyWorldInfo& worldInfo, btSoftBody** output );
    };

    class bullet2itk {
    public:
        bullet2itk( btSoftBody* softBody, VC_MeshType::Pointer output );
    };

  } // namespace meshing
} // namespace volcart

#endif // VC_USE_BULLET
