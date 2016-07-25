//
// Created by Seth Parker on 3/14/16.
//

#include "clothModelingUV.h"

namespace volcart {
    namespace texturing {

        // Constructor
        clothModelingUV::clothModelingUV( VC_MeshType::Pointer input,
                                          uint16_t unfurlIterations, uint16_t collideIterations, uint16_t expandIterations,
                                          PinIDs unfurlPins, PinIDs expansionPins ) :
                                            _mesh(input),
                                            _unfurlIterations(unfurlIterations), _collideIterations(collideIterations),
                                            _expandIterations(expandIterations), _unfurlPins(unfurlPins),
                                            _expansionPins(expansionPins)
        {
            // Default starting parameters
            _unfurlA = 10;
            _collisionA = -10;
            _expansionA = 10;

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
            btVector3 planeNormal( 0, 1, 0 );
            btCollisionShape* groundShape = new btStaticPlaneShape( planeNormal, 0); // Normal + offset along normal
            btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
            btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, groundShape, localInertia);

            _collisionPlane = new btRigidBody(cInfo);
            _collisionPlane->setUserIndex(-1);
            _World->addRigidBody(_collisionPlane);

            // Convert mesh to a softbody
            volcart::meshing::itk2bullet::itk2bullet( _mesh, _World->getWorldInfo(), &_softBody );

            // Scale the mesh so that max dimension <= 80m
            // Note: Assumes max is a positive coordinate. Not sure what this will do for small meshes
            btVector3 min, max;
            _softBody->getAabb( min, max );
            double maxDim = ( max.getX() < max.getY() ) ? max.getY() : max.getX();
            maxDim = ( maxDim < max.getZ() ) ? max.getZ() : maxDim;

            _meshToWorldScale = 80 / maxDim;
            _softBody->scale( btVector3( _meshToWorldScale, _meshToWorldScale, _meshToWorldScale) );
            _startingSurfaceArea = _SurfaceArea();

            // Set the mass for the whole cloth
            for ( int i = 0; i < _softBody->m_nodes.size(); ++i) {
                _softBody->setMass(i, 1);
            }

            // Add the softbody to the dynamics world
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
            if ( _unfurlIterations > 0 ) _unfurl();
            if ( _collideIterations > 0 ) _collide();
            if ( _expandIterations > 0 ) _expand();
        }

        // Call the stages individually
        void clothModelingUV::unfurl() { _unfurl(); };
        void clothModelingUV::collide() { _collide(); };
        void clothModelingUV::expand() { _expand(); };

        // Get UV Map created from flattened object
        volcart::UVMap clothModelingUV::getUVMap() {

            // Get the current XZ bounds of the softbody
            btVector3 min, max;
            _softBody->getAabb( min, max );

            // Round so that we have integer bounds
            double min_u = std::floor( min.getX() );
            double max_u = std::ceil ( max.getX() );
            double min_v = std::floor( min.getZ() );
            double max_v = std::ceil ( max.getZ() );

            // Scale width and height back to volume coordinates
            double aspect_width = std::abs(max_u - min_u) * ( 1/_meshToWorldScale );
            double aspect_height = std::abs(max_v - min_v) * ( 1/_meshToWorldScale );
            double aspect = aspect_width / aspect_height;

            volcart::UVMap uvMap;
            uvMap.ratio(aspect_width, aspect_height);

            // Calculate uv coordinates
            double u, v;
            for ( auto i = 0; i < _softBody->m_nodes.size(); ++i ) {
                u = ( _softBody->m_nodes[i].m_x.getX() - min_u) / (max_u - min_u);
                v = ( _softBody->m_nodes[i].m_x.getZ() - min_v) / (max_v - min_v);
                cv::Vec2d uv( u, v );

                // Add the uv coordinates into our map at the point index specified
                uvMap.set(i, uv);
            }

            return uvMap;
        }

        // Get mesh version of flattened object
        // Note: This is still in world coordinates, not volume coordinates
        VC_MeshType::Pointer clothModelingUV::getMesh() {
            VC_MeshType::Pointer output = VC_MeshType::New();
            volcart::meshing::deepCopy( _mesh, output );
            volcart::meshing::bullet2itk::bullet2itk( _softBody, output);
            return output;
        }

        ///// Simulation /////
        // Use gravity to unfurl the cloth
        void clothModelingUV::_unfurl()
        {
            // Set the simulation parameters
            _World->setInternalTickCallback( constrainMotionCallback, static_cast<void *>(this), true );
            _World->setGravity( btVector3( _unfurlA, 0, 0) );
            _softBody->getWorldInfo()->m_gravity = _World->getGravity();
            _softBody->m_cfg.kDP = 0.01; // Damping coefficient of the soft body [0,1]
            _softBody->m_materials[0]->m_kLST = 1.0; // Linear stiffness coefficient [0,1]
            _softBody->m_materials[0]->m_kAST = 1.0; // Area/Angular stiffness coefficient [0,1]
            _softBody->m_materials[0]->m_kVST = 1.0; // Volume stiffness coefficient [0,1]

            // Set the pins to not move
            for ( auto it = _unfurlPins.begin(); it != _unfurlPins.end(); ++it ) {
                _softBody->setMass( *it, 0.f );
            }

            // Run the simulation
            for ( uint16_t i = 0; i < _unfurlIterations; ++i ) {
                std::cerr << "volcart::texturing::clothUV: Unfurling " << i+1 << "/" << _unfurlIterations << "\r" << std::flush;
                _World->stepSimulation(1 / 60.f, 10);
                _softBody->solveConstraints();
            }
            std::cerr << std::endl;
        }

        // Collide the cloth with the plane
        void clothModelingUV::_collide()
        {
            // Set the simulation parameters
            _World->setInternalTickCallback( axisLockCallback, static_cast<void *>(this), true );
            _World->setGravity(btVector3(0, _collisionA, 0));
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
                std::cerr << "volcart::texturing::clothUV: Colliding " << i+1 << "/" << _collideIterations << "\r" << std::flush;
                _World->stepSimulation(1 / 60.f, 10);
                _softBody->solveConstraints();
            }
            std::cerr << std::endl;
        }

        // Expand the edges of the cloth to iron out wrinkles, then let it relax
        void clothModelingUV::_expand()
        {
            // Set the simulation parameters
            _World->setInternalTickCallback( moveTowardTargetCallback, static_cast<void *>(this), true );
            _World->setGravity(btVector3(0, 0, 0));
            _collisionPlane->setFriction(0); // (0-1] Default: 0.5
            _softBody->getWorldInfo()->m_gravity = _World->getGravity();
            _softBody->m_cfg.kDF = 0.1; // Dynamic friction coefficient (0-1] Default: 0.2
            _softBody->m_cfg.kDP = 0.01; // Damping coefficient of the soft body [0,1]
            _softBody->m_materials[0]->m_kLST = 1.0; // Linear stiffness coefficient [0,1]
            _softBody->m_materials[0]->m_kAST = 1.0; // Area/Angular stiffness coefficient [0,1]
            _softBody->m_materials[0]->m_kVST = 1.0; // Volume stiffness coefficient [0,1]

            // Get the current XZ center of the softBody
            btVector3 min, max;
            _softBody->getAabb( min, max );
            btVector3 center;
            center.setX( (max.getX() + min.getX())/2 );
            center.setY( 0 );
            center.setZ( (max.getZ() + min.getZ())/2 );

            // Setup the expansion pins
            Pin newPin;
            _currentPins.clear();
            for ( auto it = _expansionPins.begin(); it != _expansionPins.end(); ++it ) {
                newPin.index = *it;
                newPin.node = &_softBody->m_nodes[*it];
                newPin.node->m_x.setY(0); // Force the pin to the Y-plane
                newPin.target = ( newPin.node->m_x - center ) * 1.5;

                _currentPins.push_back( newPin );
            }

            // Expand the edges
            for ( uint16_t i = 0; i < _expandIterations; ++i ) {
                std::cerr << "volcart::texturing::clothUV: Expanding " << i+1 << "/" << _expandIterations << "\r" << std::flush;
                _World->stepSimulation(1 / 60.f, 10);
                _softBody->solveConstraints();
            }
            std::cerr << std::endl;

            // Relax the springs
            _World->setInternalTickCallback( axisLockCallback, static_cast<void *>(this), true );
            _World->setGravity(btVector3(0, _expansionA, 0));
            _softBody->getWorldInfo()->m_gravity = _World->getGravity();
            _softBody->m_cfg.kDP = 0.1;
            int counter = 0;
            double relativeError = std::fabs( (_startingSurfaceArea - _SurfaceArea()) / _startingSurfaceArea );
            while ( relativeError > 0.0 ) {
                std::cerr << "volcart::texturing::clothUV: Relaxing " << counter+1 << "\r" << std::flush;
                _World->stepSimulation(1 / 60.f, 10);
                _softBody->solveConstraints();

                ++counter;
                if ( counter % 10 == 0 ) relativeError = std::fabs( (_startingSurfaceArea - _SurfaceArea()) / _startingSurfaceArea );
                if ( counter >= _expandIterations * 6 ) {
                    std::cerr << std::endl << "volcart::texturing::clothUV: Warning: Max relaxation iterations reached";
                    break;
                }
            }
            std::cerr << std::endl << "volcart::texturing::clothUV: Mesh Area Relative Error: " << relativeError * 100 << "%" << std::endl;
        }

        // Set acceleration for different stages
        void clothModelingUV::setAcceleration( Stage s, double a ) {
            switch( s ) {
                case Stage::Unfurl:
                    _unfurlA = a;
                    break;
                case Stage::Collision:
                    _collisionA = a;
                    break;
                case Stage::Expansion:
                    _expansionA = a;
                    break;
                default:
                    // Should not occur
                    break;
            }
        };

        ///// Helper Functions /////

        // Calculate the surface area of the mesh using Heron's formula
        // Use the version that is stable for small angles from
        // "Miscalculating Area and Angles of a Needle-like Triangle" by Kahan
        // https://people.eecs.berkeley.edu/~wkahan/Triangle.pdf
        double clothModelingUV::_SurfaceArea() {
            double surface_area = 0;
            for(int i = 0; i < _softBody->m_faces.size(); ++i) {

                // Get the side lengths
                double a = 0, b = 0, c = 0, p = 0;
                a = _softBody->m_faces[i].m_n[0]->m_x.distance(_softBody->m_faces[i].m_n[1]->m_x);
                b = _softBody->m_faces[i].m_n[0]->m_x.distance(_softBody->m_faces[i].m_n[2]->m_x);
                c = _softBody->m_faces[i].m_n[1]->m_x.distance(_softBody->m_faces[i].m_n[2]->m_x);

                // Sort the side lengths so that a >= b >= c
                double na, nb, nc;
                nc = std::min( a, std::min( b, c ) );
                na = std::max( a, std::max( b, c ) );
                nb = a + b + c - na - nc;

                // Calculate the area
                p = (na + (nb + nc)) * (nc - (na - nb)) * (nc + (na - nb)) * (na + (nb - nc));
                double sa = 0.25 * sqrt(p);

                // Can get NaN's when using standard C++ math. Explore something like GMP
                if ( isnan(sa) ) {
                    std::cerr << std::endl << "volcart::texturing::clothUV: Warning: NaN surface area for face[" << i << "]. Evaluating as 0." << std::endl;
                    sa = 0.0;
                }
                surface_area += sa;
            }

            return surface_area;
        }

        ///// Callback Functions /////
        // Limits motion to be along the X axis only. Used in unfurl step.
        void clothModelingUV::_constrainMotion( btScalar timeStep ) {
            for ( auto n = 0; n < _softBody->m_nodes.size(); ++n )
            {
                btVector3 velocity = _softBody->m_nodes[n].m_v;
                velocity.setY(0);
                velocity.setZ(0);
                _softBody->m_nodes[n].m_v = velocity;
            }
        };
        // Apply opposite velocity to points that have "passed through" the collision plane
        void clothModelingUV::_axisLock( btScalar timeStep ) {
            for ( auto n = 0; n < _softBody->m_nodes.size(); ++n )
            {
                btVector3 pos = _softBody->m_nodes[n].m_x;
                if ( pos.getY() < 0.0 ) {
                    btVector3 velocity = _softBody->m_nodes[n].m_v;
                    velocity.setY( velocity.getY() * -1.5 ); // push it back some
                    _softBody->m_nodes[n].m_v = velocity;
                }
            }
        };
        // Move points in the _currentPins vector towards their respective targets
        void clothModelingUV::_moveTowardTarget( btScalar timeStep ) {
            for ( auto pin = _currentPins.begin(); pin < _currentPins.end(); ++pin ) {
                btVector3 delta = ( pin->target - pin->node->m_x ).normalized();
                pin->node->m_v += delta/timeStep;
            }
        };
        // This call back is used to disable other callbacks
        void clothModelingUV::_emptyPreTick( btScalar timeStep ) {
            // Don't do anything
        };

        //// Callbacks ////
        // Forward pretick callbacks to functions in a stupid Bullet physics way
        void constrainMotionCallback(btDynamicsWorld *world, btScalar timeStep) {
            clothModelingUV *w = static_cast<clothModelingUV *>( world->getWorldUserInfo() );
            w->_constrainMotion(timeStep);
        }

        void axisLockCallback(btDynamicsWorld *world, btScalar timeStep) {
            clothModelingUV *w = static_cast<clothModelingUV *>( world->getWorldUserInfo() );
            w->_axisLock(timeStep);
        }

        void moveTowardTargetCallback(btDynamicsWorld *world, btScalar timeStep) {
            clothModelingUV *w = static_cast<clothModelingUV *>( world->getWorldUserInfo() );
            w->_moveTowardTarget(timeStep);
        }

        void emptyPreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
            clothModelingUV *w = static_cast<clothModelingUV *>( world->getWorldUserInfo() );
            w->_emptyPreTick(timeStep);
        }

    } // texturing
} // volcart