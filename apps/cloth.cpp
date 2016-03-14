//
// Created by Abigail Coleman 10/28/15
//

#include <iostream>
#include <math.h>
#include <map>

#include <opencv2/opencv.hpp>

#include <vtkSmoothPolyDataFilter.h>
#include <vtkPLYReader.h>

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "volumepkg.h"
#include "io/ply2itk.h"
#include "itk2vtk.h"
#include "deepCopy.h"
#include "io/plyWriter.h"
#include "itkMeshFileReader.h"

// bullet converter
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include "itk2bullet.h"
#include <LinearMath/btVector3.h>

struct NodeTarget {
    btVector3 t_pos;
    btScalar  t_stepsize;
};

struct PinnedPoint {
    unsigned long index;
    btSoftBody::Node* node;
};

btSoftBody* global_softBody;
std::vector< PinnedPoint > pinnedPoints;
btVector3 middle(0,0,0);

btVector3 btAverageNormal( btSoftBody* body );
btScalar btAverageVelocity( btSoftBody* body );
double btSurfaceArea( btSoftBody* body );
static void planarizeCornersPreTickCallback(btDynamicsWorld *world, btScalar timeStep);
static void emptyPreTickCallback(btDynamicsWorld *world, btScalar timeStep);
static void constrainMotionCallback(btDynamicsWorld *world, btScalar timeStep);
void expandCorners(float magnitude);
void dumpState( VC_MeshType::Pointer toUpdate, btSoftBody* body, std::string suffix = "" );

void getPins(std::string path, VC_MeshType::Pointer mesh, btSoftBody *psb);

int main(int argc, char* argv[]) {
    if ( argc < 3 ) {
        std::cout << "Usage: vc_texture2 volpkg seg-id iterations" << std::endl;
        return EXIT_FAILURE;
    }

    VolumePkg vpkg = VolumePkg( argv[ 1 ] );
    std::string segID = argv[ 2 ];
    if (segID == "") {
    printf("ERROR: Incorrect/missing segmentation ID!\n");
    exit(EXIT_FAILURE);
    }
    if ( vpkg.getVersion() < 2.0) {
        printf("ERROR: Volume package is version %f but this program requires a version >= 2.0.\n", vpkg.getVersion() );
        exit(EXIT_FAILURE);
    }
    vpkg.setActiveSegmentation(segID);
    std::string meshName = vpkg.getMeshPath();

    int64_t NUM_OF_ITERATIONS = atoi( argv[ 3 ] );
    
    // Get Mesh
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName ( "decim.ply" );
    reader->Update();
    VC_MeshType::Pointer mesh = VC_MeshType::New();
    volcart::meshing::vtk2itk(reader->GetOutput(), mesh);

    // Create Dynamic world for bullet cloth simulation
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();

    btDefaultCollisionConfiguration* collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

    btSoftBodySolver* softBodySolver = new btDefaultSoftBodySolver();

    btSoftRigidDynamicsWorld* dynamicsWorld = new btSoftRigidDynamicsWorld( dispatcher,
                                                                          broadphase,
                                                                          solver,
                                                                          collisionConfiguration,
                                                                          softBodySolver);

    // Add collision plane
    btTransform startTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, -1, 0 ));
    btScalar mass = 0.f;
    btVector3 localInertia(0, 0, 0);

    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, groundShape, localInertia);

    btRigidBody* plane = new btRigidBody(cInfo);
    plane->setUserIndex(-1);
    dynamicsWorld->addRigidBody(plane);

    // convert itk mesh to bullet mesh (vertices and triangle arrays)
    btSoftBody* psb;
    volcart::meshing::itk2bullet::itk2bullet(mesh, dynamicsWorld->getWorldInfo(), &psb);
    psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RS;
    dynamicsWorld->addSoftBody(psb);
    global_softBody = psb;

    // Constraints for the mesh as a soft body
    // These needed to be tested to find optimal values.
    // Sets the mass of the whole soft body, true considers the faces along with the vertices
    // Note: Mass is in kilograms. If mass isn't high enough, nothing changes.
    printf("volcart::cloth::message: Setting mass\n");
    psb->setTotalMass( (int)(psb->m_nodes.size() * 0.01), true );

    psb->m_cfg.kDP = 0.01; // Damping coefficient of the soft body [0,1]
    psb->m_materials[0]->m_kLST = 0.25; // Linear stiffness coefficient [0,1]
    psb->m_materials[0]->m_kAST = 0.25; // Area/Angular stiffness coefficient [0,1]
    psb->m_materials[0]->m_kVST = 0.25; // Volume stiffness coefficient [0,1]

    // Load the pinned points
    getPins("pins.ply", mesh, psb);

    //////// Simulation /////////
    dynamicsWorld->setGravity(btVector3(-10, 0, 0));
    psb->getWorldInfo()->m_gravity = dynamicsWorld->getGravity(); // Have to explicitly make softbody gravity match world gravity
    dynamicsWorld->setInternalTickCallback(constrainMotionCallback, dynamicsWorld, true);

    int required_iterations = NUM_OF_ITERATIONS;
    for ( int counter = 0; counter < required_iterations; ++counter ) {
        std::cerr << "volcart::cloth::message: Step " << counter+1 << "\r" << std::flush;
        dynamicsWorld->stepSimulation(1/60.f);
        psb->solveConstraints();
    }
    dumpState( mesh, psb, "_-gravity" );


    // Flatten the original pinned points
    // Load the pinned points
    VC_MeshType::Pointer intermediate = VC_MeshType::New();
    volcart::meshing::deepCopy(mesh, intermediate);
    volcart::meshing::bullet2itk::bullet2itk(psb, intermediate);
    getPins("pins_20160312145136_-gravity.ply", intermediate, psb);

    for ( auto id = 0; id < psb->m_nodes.size(); ++id)
        psb->setMass( id, 0);

    for ( auto pin = pinnedPoints.begin(); pin != pinnedPoints.end(); ++pin ) {
        psb->setMass( pin->index, 5 );
    }

    printf("volcart::cloth::message: Fixing starting points\n");
    dynamicsWorld->setGravity(btVector3(10, 0, 0));
    psb->getWorldInfo()->m_gravity = dynamicsWorld->getGravity(); // Have to explicitly make softbody gravity match world gravity
    dynamicsWorld->setInternalTickCallback(emptyPreTickCallback, dynamicsWorld, true);
    required_iterations = 5000;
    for ( int counter = 0; counter < required_iterations; ++counter ) {
        std::cerr << "volcart::cloth::message: Step " << counter+1 << "\r" << std::flush;
        dynamicsWorld->stepSimulation(1/60.f);
        psb->solveConstraints();
    }
    dumpState( mesh, psb, "_+gravity" );

    // Add a collision plane to push the mesh onto
    btScalar min_y = psb->m_nodes[0].m_x.y();
    btScalar max_y = psb->m_nodes[0].m_x.y();
    for (size_t n_id = 1; n_id < psb->m_nodes.size(); ++n_id) {
        double _y = psb->m_nodes[n_id].m_x.y();
        double _z = psb->m_nodes[n_id].m_x.z();
        if ( _y < min_y && _z >= 0) min_y = psb->m_nodes[n_id].m_x.y();
        if ( _y > max_y && _z >= 0) max_y = psb->m_nodes[n_id].m_x.y();
    }

    btScalar plane_y = min_y - 5;

    // Set the gravity so the mesh will be pushed onto the plane
    dynamicsWorld->setGravity(btVector3(0, -10, 0));
    psb->getWorldInfo()->m_gravity = dynamicsWorld->getGravity(); // Have to explicitly make softbody gravity match world gravity

    // set the friction of the plane and the mesh s.t. the mesh can easily flatten upon collision
    plane->setFriction(0.01); // (0-1] Default: 0.5
    psb->scale( btVector3(0.01, 0.01, 0.01) );
    psb->m_cfg.kDF = 0.01; // Dynamic friction coefficient (0-1] Default: 0.2
    psb->m_cfg.kDP = 0.1; // Damping coefficient of the soft body [0,1]
    psb->setTotalMass( (int)(psb->m_nodes.size() * 0.01), true );

    // Let it settle
    printf("volcart::cloth::message: Relaxing corners\n");
    required_iterations = 25000;
    for ( int counter = 0; counter < required_iterations; ++counter ) {
        std::cerr << "volcart::cloth::message: Step " << counter+1 << "\r" << std::flush;
        dynamicsWorld->stepSimulation(1/60.f);
        //psb->solveConstraints();
    }
    std::cerr << std::endl;
    dumpState( mesh, psb, "_plane" );

    // bullet clean up
    dynamicsWorld->removeRigidBody(plane);
    delete plane->getMotionState();
    delete plane;
    delete groundShape;
    dynamicsWorld->removeSoftBody(psb);
    delete psb;
    delete dynamicsWorld;
    delete softBodySolver;
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete broadphase;

    return 0;

} // end main

///////// Get pinned points from file //////////
void getPins(std::string path, VC_MeshType::Pointer mesh, btSoftBody *psb) {

    pinnedPoints.clear();

    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName ( path.c_str() );
    reader->Update();
    VC_MeshType::Pointer pins = VC_MeshType::New();
    volcart::meshing::vtk2itk(reader->GetOutput(), pins);

    //Find the pins in the softbody
    typename VC_PointsLocatorType::Pointer pointsLocator = VC_PointsLocatorType::New();
    pointsLocator->SetPoints(mesh->GetPoints());
    pointsLocator->Initialize();

    // Iterate over all of the pins and find them in the softBody
    PinnedPoint pinnedPoint;
    for (VC_PointsInMeshIterator pin = pins->GetPoints()->Begin(); pin != pins->GetPoints()->End(); ++pin) {
        pinnedPoint.index = pointsLocator->FindClosestPoint( pins->GetPoint( pin->Index() ) );
        pinnedPoint.node = &psb->m_nodes[pinnedPoint.index];
        pinnedPoints.push_back( pinnedPoint );
    }

    // Set the pins to not move
    for ( auto pin = pinnedPoints.begin(); pin != pinnedPoints.end(); ++pin ) {
        psb->setMass( pin->index, 0 );
    }
}

btVector3 btAverageNormal(btSoftBody* body) {
    btVector3  avg_normal(0,0,0);
    for ( size_t n_id = 0; n_id < body->m_faces.size(); ++n_id ) {
        avg_normal += body->m_faces[n_id].m_normal;
    }
    avg_normal /= body->m_faces.size();
    return avg_normal;
};

btScalar btAverageVelocity(btSoftBody* body) {
    btScalar velocity = 0;
    for ( size_t n_id = 0; n_id < body->m_nodes.size(); ++n_id ) {
        velocity += body->m_nodes[n_id].m_v.length();
    }
    velocity /= body->m_nodes.size();
    return velocity;
}

// Calculate the surface area of the mesh using Heron's formula
// Let a,b,c be the lengths of the sides of a triangle and p the semiperimeter
// p = (a +  b + c) / 2
// area of triangle = sqrt( p * (p - a) * (p - b) * (p - c) )
double btSurfaceArea( btSoftBody* body ) {
    double surface_area = 0;
    for(int i = 0; i < body->m_faces.size(); ++i) {
        double a = 0, b = 0, c = 0, p = 0;
        a = body->m_faces[i].m_n[0]->m_x.distance(body->m_faces[i].m_n[1]->m_x);
        b = body->m_faces[i].m_n[0]->m_x.distance(body->m_faces[i].m_n[2]->m_x);
        c = body->m_faces[i].m_n[1]->m_x.distance(body->m_faces[i].m_n[2]->m_x);

        p = (a + b + c) / 2;

        surface_area += sqrt( p * (p - a) * (p - b) * (p - c) );
    }

    return surface_area;
}

//void planarizeCornersPreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
//    // Iterate over rigid bodies and move them towards their targets
//    for( size_t p_id = 0; p_id < pinnedPoints.size(); ++p_id ) {
//        if ( pinnedPoints[p_id]->m_x == targetPoints[p_id].t_pos ) continue;
//        btVector3 delta = (targetPoints[p_id].t_pos - pinnedPoints[p_id]->m_x).normalized() * targetPoints[p_id].t_stepsize;
//        pinnedPoints[p_id]->m_v += delta/timeStep;
//    }
//};

void constrainMotionCallback(btDynamicsWorld *world, btScalar timeStep) {

    for ( auto n = 0; n < global_softBody->m_nodes.size(); ++n ) {
        btVector3 velocity = global_softBody->m_nodes[n].m_v;
        velocity.setY(0);
        velocity.setZ(0);
        global_softBody->m_nodes[n].m_v = velocity;
    }
};

void emptyPreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
    // This call back is used to disable other callbacks
    // Particularly used for relaxing the four corners
};

//void expandCorners(float magnitude) {
//    btScalar stepSize = 1;
//    btScalar _magnitude = magnitude;
//    for( size_t p_id = 0; p_id < pinnedPoints.size(); ++p_id ) {
//        targetPoints[p_id].t_pos += (targetPoints[p_id].t_pos - middle).normalized() * _magnitude;
//        targetPoints[p_id].t_stepsize = stepSize;
//    }
//}

void dumpState( VC_MeshType::Pointer toUpdate, btSoftBody* body, std::string suffix ) {
    VC_MeshType::Pointer output = VC_MeshType::New();
    volcart::meshing::deepCopy(toUpdate, output);
    volcart::meshing::bullet2itk::bullet2itk(body, output);

    std::string path = "inter_" + VC_DATE_TIME() + suffix + ".obj";
    volcart::io::objWriter writer(path, output);
    writer.write();
}
