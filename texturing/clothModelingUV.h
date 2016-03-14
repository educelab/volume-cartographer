//
// Created by Media Team on 3/14/16.
//

#ifndef VC_CLOTHMODELINGUV_H
#define VC_CLOTHMODELINGUV_H

#include <iostream>

#include <opencv2/opencv.hpp>
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "itk2bullet.h"
#include "deepCopy.h"

namespace volcart {
    namespace texturing {

        // Pretick callbacks
        static void constrainMotionCallback(btDynamicsWorld *world, btScalar timeStep);
        static void emptyPreTickCallback(btDynamicsWorld *world, btScalar timeStep);

        class clothModelingUV {
        public:
            typedef std::vector< unsigned long > PinIDs;

            struct NodeTarget {
                btVector3 t_pos;
                btScalar  t_stepsize;
            };

            struct Pin {
                unsigned long index;
                btSoftBody::Node* node;
                NodeTarget target;
            };

            clothModelingUV( VC_MeshType::Pointer input,
                             uint16_t unfurlIterations, uint16_t collideIterations, uint16_t expandIterations,
                             cv::Vec3d plane_normal, PinIDs unfurlPins, PinIDs expansionPins);
            ~clothModelingUV();

            void run();

            // Output
            VC_MeshType::Pointer getMesh();
            volcart::UVMap getUVMap();

            //Callback functionality
            void _constrainMotion( btScalar timeStep );
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
            uint16_t _unfurlIterations;
            PinIDs        _unfurlPins;
            void          _unfurl();

            uint16_t _collideIterations;
            void          _collide();

            uint16_t _expandIterations;
            PinIDs        _expansionPins;
            void          _expand();

            //Simulation World
            btBroadphaseInterface* _WorldBroadphase;
            btDefaultCollisionConfiguration* _WorldCollisionConfig;
            btCollisionDispatcher* _WorldCollisionDispatcher;
            btSequentialImpulseConstraintSolver* _WorldConstraintSolver;
            btSoftBodySolver* _WorldSoftBodySolver;
            btSoftRigidDynamicsWorld* _World;
        };

        // Forward pretick callbacks to functions in a stupid Bullet physics way
        void constrainMotionCallback(btDynamicsWorld *world, btScalar timeStep) {
            clothModelingUV *w = static_cast<clothModelingUV *>( world->getWorldUserInfo() );
            w->_constrainMotion(timeStep);
        }

        void emptyPreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
            clothModelingUV *w = static_cast<clothModelingUV *>( world->getWorldUserInfo() );
            w->_emptyPreTick(timeStep);
        }
    }
}

#endif //VC_CLOTHMODELINGUV_H
