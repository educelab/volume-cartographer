// Cloth-modelling parameterization
// Created by Seth Parker on 3/14/16.

//   This algorithm treats 3D->2D parameterization as a cloth modeling/softbody dynamics problem. The mesh being
// flattened can be thought of as a "wrinkled sheet" in 3D space. First, two corners of this "sheet" are pinned in place
// and the rest of the "sheet" is allowed to unfurl relative to these points. Second, this "sheet" is then dropped on a
// collision plane, which helps to smooth out many of the largely curved sections. Third, the boundary of the "sheet" is
// stretched "outward" and then allowed to relax to a resting position that minimizes the error in the surface area of
// the sheet.
//   The results of this method are highly dependent on the input mesh and the large number of parameters that can be
// tweaked. I don't think there's much use for this class as is, but I'm leaving it here for reference. The general
// Bullet physics framework will be useful for improving the results of parameterizations generated by other methods.

#ifndef VC_CLOTHMODELINGUV_H
#define VC_CLOTHMODELINGUV_H

#ifdef USE_BULLET

#include <iostream>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>

#include "common/vc_defines.h"
#include "common/types/UVMap.h"
#include "meshing/itk2bullet.h"
#include "meshing/deepCopy.h"

namespace volcart {
    namespace texturing {
        // Pretick callbacks
        static void constrainMotionCallback(btDynamicsWorld *world, btScalar timeStep);
        static void axisLockCallback(btDynamicsWorld *world, btScalar timeStep);
        static void moveTowardTargetCallback(btDynamicsWorld *world, btScalar timeStep);
        static void emptyPreTickCallback(btDynamicsWorld *world, btScalar timeStep);

        class ClothModelingUVMapping {
        public:
            typedef std::vector< unsigned long > PinIDs;
            enum Stage { Unfurl, Collision, Expansion };

            struct Pin {
                unsigned long index;
                btSoftBody::Node* node;
                btVector3 target;
            };

            ClothModelingUVMapping( VC_MeshType::Pointer input,
                             uint16_t unfurlIterations, uint16_t collideIterations, uint16_t expandIterations,
                             PinIDs unfurlPins, PinIDs expansionPins);
            ~ClothModelingUVMapping();

            // Run the simulation all at once or in stages
            void run();
            void unfurl();
            void collide();
            void expand();

            // Parameters
            void setAcceleration( Stage s, double a );

            // Output
            VC_MeshType::Pointer getMesh();
            volcart::UVMap getUVMap();

            //Callback functionality
            void _constrainMotion( btScalar timeStep );
            void _axisLock( btScalar timeStep );
            void _moveTowardTarget( btScalar timeStep );
            void _emptyPreTick( btScalar timeStep );

        private:
            // Softbody
            const VC_MeshType::Pointer _mesh;
            btSoftBody* _softBody;
            double _meshToWorldScale;
            std::vector< Pin > _currentPins;

            // Collision Plane
            btRigidBody* _collisionPlane;

            // Simulation
            uint16_t   _unfurlIterations;
            double     _unfurlA;
            PinIDs     _unfurlPins;
            void       _unfurl();

            uint16_t _collideIterations;
            double   _collisionA;
            void     _collide();

            uint16_t _expandIterations;
            double   _expansionA;
            PinIDs   _expansionPins;
            void     _expand();

            // Helpers
            double _startingSurfaceArea;
            double _SurfaceArea();

            // Simulation World
            btBroadphaseInterface* _WorldBroadphase;
            btDefaultCollisionConfiguration* _WorldCollisionConfig;
            btCollisionDispatcher* _WorldCollisionDispatcher;
            btSequentialImpulseConstraintSolver* _WorldConstraintSolver;
            btSoftBodySolver* _WorldSoftBodySolver;
            btSoftRigidDynamicsWorld* _World;
        };

        //// Callbacks ////
        void constrainMotionCallback(btDynamicsWorld *world, btScalar timeStep);
        void axisLockCallback(btDynamicsWorld *world, btScalar timeStep);
        void moveTowardTargetCallback(btDynamicsWorld *world, btScalar timeStep);
        void emptyPreTickCallback(btDynamicsWorld *world, btScalar timeStep);
    }
}

#endif // USE_BULLET

#endif //VC_CLOTHMODELINGUV_H
