//
// Created by Abigail Coleman 10/28/15
//

#include <iostream>
#include <math.h>

#include <opencv2/opencv.hpp>

#include "volumepkg.h"
#include "vc_defines.h"
#include "io/ply2itk.h"
#include "io/objWriter.h"

// bullet converter
#include "itk2bullet.h"

struct NodeTarget {
    btVector3 t_pos;
    btScalar  t_stepsize;
};

bool btIsStatic(btSoftBody* body);
static void softBodyTickCallback(btDynamicsWorld *world, btScalar timeStep);
std::vector<btSoftBody::Node*> pinnedPoints;
std::vector<NodeTarget> targetPoints;

int main(int argc, char* argv[]) {
    if ( argc < 5 ) {
        std::cout << "Usage: vc_texture2 volpkg seg-id iterations" << std::endl;
    }

    VolumePkg vpkg = VolumePkg( argv[ 1 ] );
    std::string segID = argv[ 2 ];
    if (segID == "") {
    std::cerr << "ERROR: Incorrect/missing segmentation ID!" << std::endl;
    exit(EXIT_FAILURE);
    }
    if ( vpkg.getVersion() < 2.0) {
    std::cerr << "ERROR: Volume package is version " << vpkg.getVersion() << " but this program requires a version >= 2.0."  << std::endl;
    exit(EXIT_FAILURE);
    }
    vpkg.setActiveSegmentation(segID);
    std::string meshName = vpkg.getMeshPath();

    int NUM_OF_ITERATIONS = atoi( argv[ 3 ] );

    // declare pointer to new Mesh object
    VC_MeshType::Pointer  mesh = VC_MeshType::New();

    int meshWidth = -1;
    int meshHeight = -1;

    // try to convert the ply to an ITK mesh
    if (!volcart::io::ply2itkmesh(meshName, mesh, meshWidth, meshHeight)) {
        exit( -1 );
    };

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

    dynamicsWorld->setGravity(btVector3(0, 0, 0));

    // convert itk mesh to bullet mesh (vertices and triangle arrays)
    std::cerr << "volcart::cloth::message: Converting mesh to softBody" << std::endl;
    btSoftBody* psb;
    volcart::meshing::itk2bullet::itk2bullet(mesh, dynamicsWorld->getWorldInfo(), &psb);

    psb->getWorldInfo()->m_gravity = dynamicsWorld->getGravity(); // Have to explicitly make softbody gravity match world gravity
    dynamicsWorld->addSoftBody(psb);

    // Constraints for the mesh as a soft body
    // These needed to be tested to find optimal values.
    // Sets the mass of the whole soft body, true considers the faces along with the vertices
    // Note: Mass is in kilograms. If mass isn't high enough, nothing changes.
    std::cerr << "volcart::cloth::message: Setting mass" << std::endl;
    // psb->setTotalMass( (int)(psb->m_nodes.size() * 0.001), true );
    psb->setTotalMass(10, true );

    psb->m_cfg.kDP = 0.1; // Damping coefficient of the soft body [0,1]
    psb->m_materials[0]->m_kLST = 1.0; // Linear stiffness coefficient [0,1]
    psb->m_materials[0]->m_kAST = 1.0; // Area/Angular stiffness coefficient [0,1]
    psb->m_materials[0]->m_kVST = 1.0; // Volume stiffness coefficient [0,1]

    // Initialize the four rigid bodies to be attached to the corners
    btCollisionShape* fallShape = new btSphereShape(1);
    btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
    btScalar mass = 10;
    btVector3 fallInertia(0, 0, 0);
    fallShape->calculateLocalInertia(mass, fallInertia);
    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);

    btRigidBody* top_left = new btRigidBody(fallRigidBodyCI);
    dynamicsWorld->addRigidBody(top_left);
    btRigidBody* top_right = new btRigidBody(fallRigidBodyCI);
    dynamicsWorld->addRigidBody(top_right);
    btRigidBody* bottom_left = new btRigidBody(fallRigidBodyCI);
    dynamicsWorld->addRigidBody(bottom_left);
    btRigidBody* bottom_right = new btRigidBody(fallRigidBodyCI);
    dynamicsWorld->addRigidBody(bottom_right);

    // Find the position of the four corner nodes
    // Currently assumes that the first point has the same z-value as the rest of the starting chain
    int min_z = (int) std::floor(mesh->GetPoint(0)[2]);
    int chain_size = 0;
    // Calculate chain length
    for(int i = 0; i < psb->m_nodes.size(); ++i) {
        if( (int)psb->m_nodes[i].m_x.z() <= min_z )
            ++chain_size;
        else
            break;
    }

    // For debug. These values should match.
    if ( chain_size != meshWidth ) return EXIT_FAILURE;

    // Append rigid bodies to respective nodes of mesh
    // Assumes the chain length is constant throughout the mesh
    psb->appendAnchor(0, top_left);
    psb->appendAnchor(chain_size - 1 , top_right);
    psb->appendAnchor(psb->m_nodes.size() - chain_size, bottom_left);
    psb->appendAnchor(psb->m_nodes.size() - 1, bottom_right);

    // Calculate the surface area of the mesh using Heron's formula
    // Let a,b,c be the lengths of the sides of a triangle and p the semiperimeter
    // p = (a +  b + c) / 2
    // area of triangle = sqrt( p * (p - a) * (p - b) * (p - c) )
    double surface_area = 0;
    for(int i = 0; i < psb->m_faces.size(); ++i) {
    	double a = 0, b = 0, c = 0, p = 0;
    	a = psb->m_faces[i].m_n[0]->m_x.distance(psb->m_faces[i].m_n[1]->m_x);
    	b = psb->m_faces[i].m_n[0]->m_x.distance(psb->m_faces[i].m_n[2]->m_x);
    	c = psb->m_faces[i].m_n[1]->m_x.distance(psb->m_faces[i].m_n[2]->m_x);

    	p = (a + b + c) / 2;

    	surface_area += sqrt( p * (p - a) * (p - b) * (p - c) );
    }


    // // Create target positions for our rigid bodies
    // for( auto p_id = pinnedPoints.begin(); p_id < pinnedPoints.end(); ++p_id ) {
    //     NodeTarget n_target;
    //     btVector3 target_pos;
    //     btScalar distance;
    //     btScalar t_x, t_y, t_z;

    //     if ( p_id == pinnedPoints.begin() ) {
    //         target_pos = (*p_id)->m_x;
    //         target_pos.setZ((max_y + min_y) / 2);
    //     }
    //     else {
    //         // Aligns the pinned points parallel with the x plane
    //         distance = (*p_id)->m_x.distance((*std::prev(p_id))->m_x); //wtf
    //         t_x = targetPoints.back().t_pos.getX() + distance;
    //         t_y = (*p_id)->m_x.getY();
    //         t_z = (max_y + min_y) / 2;
    //         target_pos = btVector3(t_x, t_y, t_z);
    //     }

    //     n_target.t_pos = target_pos;
    //     n_target.t_stepsize = (*p_id)->m_x.distance(target_pos) / 60; // Will take minimum 60 iterations to reach target
    //     targetPoints.push_back(n_target);
    // }

    // step simulation
    std::cerr << "volcart::cloth::message: Beginning simulation" << std::endl;
    for (int i = 0; i < NUM_OF_ITERATIONS; ++i) {
        std::cerr << "volcart::cloth::message: Step " << i + 1 << "/" << NUM_OF_ITERATIONS << "\r" << std::flush;
        dynamicsWorld->stepSimulation(1/60.f);
        psb->solveConstraints();
    }

    std::cerr << std::endl;

    // Convert soft body to itk mesh
    volcart::meshing::bullet2itk::bullet2itk(mesh, psb);

    volcart::io::objWriter objwriter("cloth.obj", mesh);

    objwriter.write();

    // bullet clean up
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

bool btIsStatic(btSoftBody* body) {
    btVector3  avg_normal(0,0,0);
    for ( size_t n_id = 0; n_id < body->m_faces.size(); ++n_id ) {
        avg_normal += body->m_faces[n_id].m_normal;
    }
    avg_normal /= body->m_faces.size();
    bool result = ( avg_normal.absolute().getZ() > 0.9 );
    return result;
};

void softBodyTickCallback(btDynamicsWorld *world, btScalar timeStep) {
    // Iterate over rigid bodies and move them towards their targets
    for( size_t p_id = 0; p_id < pinnedPoints.size(); ++p_id ) {
        if ( pinnedPoints[p_id]->m_x == targetPoints[p_id].t_pos ) continue;
        btVector3 delta = (targetPoints[p_id].t_pos - pinnedPoints[p_id]->m_x).normalized() * targetPoints[p_id].t_stepsize;
        pinnedPoints[p_id]->m_x += delta;
        pinnedPoints[p_id]->m_v += delta/timeStep;
    }
};
