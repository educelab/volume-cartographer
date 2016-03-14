//
// Created by Media Team on 3/14/16.
//

#include "clothModelingUV.h"

namespace volcart {
    namespace texturing {

        // Constructor
        clothModelingUV::clothModelingUV( VC_MeshType::Pointer input,
                                          uint16_t unfurlIterations, uint16_t collideIterations, uint16_t expandIterations,
                                          cv::Vec3d plane_normal, PinIDs unfurlPins, PinIDs expansionPins ) :
                                            _mesh(input),
                                            _unfurlIterations(unfurlIterations), _unfurlPins(unfurlPins),
                                            _collideIterations(collideIterations),
                                            _expandIterations(expandIterations), _expansionPins(expansionPins)
        {

            // Create Dynamic world for bullet cloth simulation
            _WorldBroadphase = new btDbvtBroadphase();
            _WorldCollisionConfig = new btSoftBodyRigidBodyCollisionConfiguration();
            _WorldCollisionDispatcher = new btCollisionDispatcher(_WorldCollisionConfig);
            _WorldConstraintSolver = new btSequentialImpulseConstraintSolver();
            _WorldSoftBodySolver = new btDefaultSoftBodySolver();
            _World = new btSoftRigidDynamicsWorld( _WorldCollisionDispatcher,
                                                   _WorldBroadphase,
                                                   _WorldConstraintSolver,
                                                   _WorldCollisionConfig,
                                                   _WorldSoftBodySolver );

            // Add the collision plane at the origin
            btTransform startTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, 0, 0 ));
            btScalar mass = 0.f;
            btVector3 localInertia(0, 0, 0);
            btVector3 planeNormal( plane_normal(0), plane_normal(1), plane_normal(2) );
            btCollisionShape* groundShape = new btStaticPlaneShape( planeNormal, 0); // Normal + offset along normal
            btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
            btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, groundShape, localInertia);

            _collisionPlane = new btRigidBody(cInfo);
            _collisionPlane->setUserIndex(-1);
            _World->addRigidBody(_collisionPlane);

            // Softbody
            volcart::meshing::itk2bullet::itk2bullet( _mesh, _World->getWorldInfo(), &_softBody );

            // Scale the mesh so that max dimension <= 80m
            // Note: Assumes max is a positive coordinate. Not sure what this will do for small meshes
            btVector3 min, max;
            _softBody->getAabb( min, max );
            double maxDim = ( max.getX() < max.getY() ) ? max.getY() : max.getX();
            maxDim = ( maxDim < max.getZ() ) ? max.getZ() : maxDim;

            _meshToWorldScale = 80 / maxDim;
            _softBody->scale( btVector3( _meshToWorldScale, _meshToWorldScale, _meshToWorldScale) );

            // Set the mass for the whole cloth
            for ( int i = 0; i < _softBody->m_nodes.size(); ++i) {
                _softBody->setMass(i, 1);
            }

            // Add the softbody
            _softBody->randomizeConstraints();
            _softBody->updateNormals();
            _World->addSoftBody( _softBody );
        };

        // Destructor
        clothModelingUV::~clothModelingUV() {
            _World->removeSoftBody( _softBody );
            delete _softBody;

            _World->removeRigidBody( _collisionPlane );
            delete _collisionPlane->getMotionState();
            delete _collisionPlane;

            delete _World;
            delete _WorldConstraintSolver;
            delete _WorldSoftBodySolver;
            delete _WorldCollisionConfig;
            delete _WorldCollisionDispatcher;
            delete _WorldBroadphase;
        }

        // Process this mesh
        void clothModelingUV::run() {
            _unfurl();
        }

        // Get output
        VC_MeshType::Pointer clothModelingUV::getMesh() {
            VC_MeshType::Pointer output = VC_MeshType::New();
            volcart::meshing::deepCopy( _mesh, output );
            volcart::meshing::bullet2itk::bullet2itk( _softBody, output);

            return output;
        }

        // Simulation
        void clothModelingUV::_unfurl()
        {
            // Set the simulation parameters
            _World->setInternalTickCallback( constrainMotionCallback, static_cast<void *>(this), true );
            _World->setGravity( btVector3(-10, 0, 0) );
            _softBody->getWorldInfo()->m_gravity = _World->getGravity();
            _softBody->m_cfg.kDP = 0.01; // Damping coefficient of the soft body [0,1]
            _softBody->m_materials[0]->m_kLST = 0.2; // Linear stiffness coefficient [0,1]
            _softBody->m_materials[0]->m_kAST = 0.2; // Area/Angular stiffness coefficient [0,1]
            _softBody->m_materials[0]->m_kVST = 0.2; // Volume stiffness coefficient [0,1]

            // Set the pins to not move
            for ( auto it = _unfurlPins.begin(); it != _unfurlPins.end(); ++it ) {
                _softBody->setMass( *it, 0.f );
            }

            // Run the simulation
            for ( uint16_t i = 0; i < _unfurlIterations; ++i ) {
                std::cerr << "volcart::texturing::clothUV: Unfurling " << i+1 << "/" << _unfurlIterations << std::flush;
                _World->stepSimulation(1 / 60.f, 10);
                _softBody->solveConstraints();
            }
            std::cerr << std::endl;
        }

        void clothModelingUV::_collide()
        {
            // Set the simulation parameters
            _World->setInternalTickCallback( emptyPreTickCallback, static_cast<void *>(this), true );
            _World->setGravity(btVector3(0, -10, 0));
            _collisionPlane->setFriction(0); // (0-1] Default: 0.5
            _softBody->getWorldInfo()->m_gravity = _World->getGravity();
            _softBody->m_cfg.kDF = 0.1; // Dynamic friction coefficient (0-1] Default: 0.2
            _softBody->m_cfg.kDP = 0.01; // Damping coefficient of the soft body [0,1]

            // Reset all pins to move

            for ( auto n = 0; n < _softBody->m_nodes.size(); ++n ) {
                _softBody->setMass( n, 1 );
            }

            // Run the simulation
            for ( uint16_t i = 0; i < _collideIterations; ++i ) {
                std::cerr << "volcart::texturing::clothUV: Colliding " << i+1 << "/" << _collideIterations << std::flush;
                _World->stepSimulation(1 / 60.f, 10);
                _softBody->solveConstraints();
            }
            std::cerr << std::endl;
        }

        ///// Callback Functions /////
        void clothModelingUV::_constrainMotion( btScalar timeStep ) {
            for ( auto n = 0; n < _softBody->m_nodes.size(); ++n )
            {
                btVector3 velocity = _softBody->m_nodes[n].m_v;
                velocity.setZ(0);
                _softBody->m_nodes[n].m_v = velocity;
            }
        };

        void clothModelingUV::_emptyPreTick( btScalar timeStep ) {
            // This call back is used to disable other callbacks
        };

    }
}