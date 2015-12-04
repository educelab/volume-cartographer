//
// Created by Abigail Coleman 10/28/15
//

#include <iostream>

#include <opencv2/opencv.hpp>

#include "volumepkg.h"
#include "vc_defines.h"
#include "io/ply2itk.h"
#include "io/objWriter.h"

// bullet converter
#include "itk2bullet.h"

static void softBodyTickCallback(btDynamicsWorld *world, btScalar timeStep);
std::vector<btSoftBody::Node*> pinnedPoints;
std::vector<btVector3> targetPoints;

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

    dynamicsWorld->setGravity(btVector3(0, -10, 0));
    dynamicsWorld->setInternalTickCallback(softBodyTickCallback, dynamicsWorld, true);

    // convert itk mesh to bullet mesh (vertices and triangle arrays)
    std::cerr << "volcart::cloth::message: Converting mesh to softBody" << std::endl;
    btSoftBody* psb;
    volcart::meshing::itk2bullet::itk2bullet(mesh, dynamicsWorld->getWorldInfo(), &psb);

    dynamicsWorld->addSoftBody(psb);

    // Constraints for the mesh as a soft body
    // These needed to be tested to find optimal values.
    // Sets the mass of the whole soft body, true considers the faces along with the vertices
    // Note: Mass is in kilograms. If mass isn't high enough, nothing changes.
    std::cerr << "volcart::cloth::message: Setting mass" << std::endl;
    // psb->setTotalMass( (int)(psb->m_nodes.size() * 0.001), true );
    psb->setTotalMass(10, true );

    // set the damping coefficient of the soft body [0,1]
    psb->m_cfg.kDP = 0.5;

    // Set the top row of vertices such that they wont move/fall
    // Currently assumes that the first point has the same z-value as the rest of the starting chain
    int min_z = mesh->GetPoint(0)[2];
    int min_y = 100000;
    int max_y = 0;
    std::cerr << "volcart::cloth::message: Pinning points at Z: " << min_z << std::endl;
    for(unsigned long i = 0; i < psb->m_nodes.size(); ++i) {
        if( (int)psb->m_nodes[i].m_x.z() <= min_z) {
            psb->setMass(i, 0);
            btSoftBody::Node* node_ptr = &psb->m_nodes[i];
            pinnedPoints.push_back(node_ptr);

            if( (int)psb->m_nodes[i].m_x.y() > max_y )
            	max_y = (int)psb->m_nodes[i].m_x.y();
            if ( (int)psb->m_nodes[i].m_x.y() < min_y )
            	min_y = (int)psb->m_nodes[i].m_x.y();
        }
    }

    // rotate mesh 90 degrees around the y axis
    std::cerr << "volcart::cloth::message: Rotating mesh" << std::endl;
    psb->rotate(btQuaternion(0,SIMD_PI/2,0));

    // Create target positions for our pinned points
    for( auto p_id = pinnedPoints.begin(); p_id < pinnedPoints.end(); ++p_id ) {
        btVector3 target_pos;
        btScalar distance;
        btScalar t_x, t_y, t_z;

        if ( p_id == pinnedPoints.begin() ) {
            target_pos = (*p_id)->m_x;
            target_pos[2] = (max_y + min_y) / 2;
        }
        else {
        	// Aligns the pinned points parallel with the x plane
            distance = (*p_id)->m_x.distance((*std::prev(p_id))->m_x); //wtf
            t_x = targetPoints.back().getX() + distance;
            t_y = (*p_id)->m_x.getY();
            // t_z = targetPoints.back().getZ();
            t_z = (max_y + min_y) / 2;
            target_pos = btVector3(t_x, t_y, t_z);
        }

        targetPoints.push_back(target_pos);
    }

    // step simulation
    std::cerr << "volcart::cloth::message: Beginning simulation" << std::endl;
    for (int i = 0; i < NUM_OF_ITERATIONS; ++i) {
        std::cerr << "volcart::cloth::message: Step " << i + 1 << "/" << NUM_OF_ITERATIONS << "\r" << std::flush;
        dynamicsWorld->stepSimulation(1/ 60.f);
        psb->solveConstraints();
    }

    std::cerr << std::endl;

    // rotate mesh back to original orientation
    psb->rotate(btQuaternion(0,-SIMD_PI/2,0));

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

void softBodyTickCallback(btDynamicsWorld *world, btScalar timeStep) {
    for( size_t p_id = 0; p_id < pinnedPoints.size(); ++p_id ) {
        btVector3 delta = (targetPoints[p_id] - pinnedPoints[p_id]->m_x) * timeStep;
        pinnedPoints[p_id]->m_x += delta;
        pinnedPoints[p_id]->m_v += delta/timeStep;
    }
};