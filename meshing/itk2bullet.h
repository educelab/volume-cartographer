//
// Created by Abigail Coleman 10/21/15
//

#ifndef VC_ITK2BULLET_H
#define VC_ITK2BULLET_H

#include "vc_defines.h"
#include "LinearMath/btScalar.h"

// Bullet Soft Body
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>

namespace volcart {
  namespace meshing {

    class itk2bullet {
    public:
    	itk2bullet( VC_MeshType::Pointer input, btScalar vertices[], int faces[][3] );
    };

    class bullet2itk {
    public:
    	bullet2itk( VC_MeshType::Pointer output, btSoftBody* softBody);
    };

  } // namespace meshing
} // namespace volcart

#endif //VC_ITK2BULLET_H