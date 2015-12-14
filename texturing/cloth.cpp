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
double btSurfaceArea( btSoftBody* body );
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

    dynamicsWorld->setInternalTickCallback(softBodyTickCallback, dynamicsWorld, true);

    // Find the position of the four corner nodes
    // Currently assumes that the first point has the same z-value as the rest of the starting chain
    int min_z = (int) std::floor(mesh->GetPoint(0)[2]);
    int chain_size = 0;
    double chain_length = 0;
    btVector3 previous_node;
    // Calculate chain size and chain length
    for(int i = 0; i < psb->m_nodes.size(); ++i) {
        if( (int)psb->m_nodes[i].m_x.z() <= min_z ) {
        	if (chain_size == 0)
        		previous_node = psb->m_nodes[i].m_x;
        	else {
        		chain_length += psb->m_nodes[i].m_x.distance(previous_node);
        		previous_node = psb->m_nodes[i].m_x;
        	}
            ++chain_size;
        }
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

    // Calculate the surface area of the mesh
    double surface_area = btSurfaceArea(psb);

    std::cout << "Chain size: " << chain_size << " | Chain Length: " << chain_length << " | Surface area: " << surface_area << std::endl;

    // Create target positions with step size for our four corners
    NodeTarget n_target;
    btVector3 target_pos;
    btScalar distance;
    btScalar t_x, t_y, t_z;
    btSoftBody::Node* node_ptr = &psb->m_nodes[0];
    pinnedPoints.push_back(node_ptr);

    // top left corner
    target_pos = psb->m_nodes[0].m_x;
    n_target.t_pos = target_pos;
    n_target.t_stepsize = (node_ptr)->m_x.distance(target_pos) / 60; // Will take minimum 60 iterations to reach target
    targetPoints.push_back(n_target);

    // top right corner
    node_ptr = &psb->m_nodes[chain_size - 1];
    pinnedPoints.push_back(node_ptr);
    t_x = psb->m_nodes[0].m_x.x() + chain_length;
    t_y = psb->m_nodes[0].m_x.y();
    t_z = psb->m_nodes[0].m_x.z();
    target_pos = btVector3(t_x, t_y, t_z);
    n_target.t_pos = target_pos;
    n_target.t_stepsize = (node_ptr)->m_x.distance(target_pos) / 60; // Will take minimum 60 iterations to reach target
    targetPoints.push_back(n_target);

    // bottom left corner
    node_ptr = &psb->m_nodes[psb->m_nodes.size() - chain_size];
    pinnedPoints.push_back(node_ptr);
    t_x = psb->m_nodes[0].m_x.x();
    t_y = psb->m_nodes[0].m_x.y();
    t_z = psb->m_nodes[0].m_x.z() + (surface_area / chain_length);
    target_pos = btVector3(t_x, t_y, t_z);
    n_target.t_pos = target_pos;
    n_target.t_stepsize = (node_ptr)->m_x.distance(target_pos) / 60; // Will take minimum 60 iterations to reach target
    targetPoints.push_back(n_target);

    // bottom right corner
    node_ptr = &psb->m_nodes[psb->m_nodes.size() - 1];
    pinnedPoints.push_back(node_ptr);
    t_x = psb->m_nodes[0].m_x.x() + chain_length;
    t_y = psb->m_nodes[0].m_x.y();
    t_z = psb->m_nodes[0].m_x.z() + (surface_area / chain_length);
    target_pos = btVector3(t_x, t_y, t_z);
    n_target.t_pos = target_pos;
    n_target.t_stepsize = (node_ptr)->m_x.distance(target_pos) / 60; // Will take minimum 60 iterations to reach target
    targetPoints.push_back(n_target);

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

void softBodyTickCallback(btDynamicsWorld *world, btScalar timeStep) {
	// size_t p_id = 0;
	// btVector3 delta = (targetPoints[p_id].t_pos - pinnedPoints[p_id]->m_x).normalized() * targetPoints[p_id].t_stepsize;
	// pinnedPoints[p_id]->m_x += delta;
 //    pinnedPoints[p_id]->m_v += delta/timeStep;
	// top_left->activate(true);
 //    top_left->translate(delta);

 //    ++p_id;
 //    delta = (targetPoints[p_id].t_pos - pinnedPoints[p_id]->m_x).normalized() * targetPoints[p_id].t_stepsize;
	// pinnedPoints[p_id]->m_x += delta;
 //    pinnedPoints[p_id]->m_v += delta/timeStep;
 //    top_right->activate(true);
 //    top_right->translate(delta);

 //    ++p_id;
 //    delta = (targetPoints[p_id].t_pos - pinnedPoints[p_id]->m_x).normalized() * targetPoints[p_id].t_stepsize;
	// pinnedPoints[p_id]->m_x += delta;
 //    pinnedPoints[p_id]->m_v += delta/timeStep;
 //    bottom_left->activate(true);
 //    bottom_left->translate(delta);

 //    ++p_id;
 //    delta = (targetPoints[p_id].t_pos - pinnedPoints[p_id]->m_x).normalized() * targetPoints[p_id].t_stepsize;
	// pinnedPoints[p_id]->m_x += delta;
 //    pinnedPoints[p_id]->m_v += delta/timeStep;
 //    bottom_right->activate(true);
 //    bottom_right->translate(delta);

    // Iterate over rigid bodies and move them towards their targets
    for( size_t p_id = 0; p_id < pinnedPoints.size(); ++p_id ) {
        if ( pinnedPoints[p_id]->m_x == targetPoints[p_id].t_pos ) continue;
        btVector3 delta = (targetPoints[p_id].t_pos - pinnedPoints[p_id]->m_x).normalized() * targetPoints[p_id].t_stepsize;
        pinnedPoints[p_id]->m_x += delta;
        pinnedPoints[p_id]->m_v += delta/timeStep;
    }
};
