//
// Created by Media Team on 3/14/16.
//

#include <iostream>

#include <vtkPLYReader.h>

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>

#include "vc_defines.h"
#include "shapes.h"
#include "itk2bullet.h"
#include "itk2vtk.h"
#include "deepCopy.h"
#include "io/objWriter.h"

void dumpState( VC_MeshType::Pointer toUpdate, btSoftBody* body, std::string suffix = "" );

int main( int argc, char* argv[] ) {

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

    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, groundShape, localInertia);

    btRigidBody* plane = new btRigidBody(cInfo);
    plane->setFriction(0); // (0-1] Default: 0.5
    plane->setUserIndex(-1);
    dynamicsWorld->addRigidBody(plane);


    // Softbody
    // Get Mesh
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName ( "collision.ply" );
    reader->Update();
    VC_MeshType::Pointer mesh = VC_MeshType::New();
    volcart::meshing::vtk2itk(reader->GetOutput(), mesh);
    btSoftBody* psb;
    volcart::meshing::itk2bullet::itk2bullet( mesh, dynamicsWorld->getWorldInfo(), &psb);
    for ( int i = 0; i < psb->m_nodes.size(); ++i) {
        psb->setMass(i, 1);
    }
    psb->scale( btVector3(0.01, 0.01, 0.01) );
    psb->randomizeConstraints();
    psb->updateNormals();
    psb->m_cfg.kDF = 0.1; // Dynamic friction coefficient (0-1] Default: 0.2
    psb->m_cfg.kDP = 0.01; // Damping coefficient of the soft body [0,1]
    dynamicsWorld->addSoftBody(psb);

    dynamicsWorld->setGravity(btVector3(0, -20, 0));
    psb->getWorldInfo()->m_gravity = dynamicsWorld->getGravity();


    for (int i = 0; i < 200; i++) {
        dynamicsWorld->stepSimulation(1 / 60.f, 10);
        psb->solveConstraints();

        std::cout << "sphere height: " << psb->m_nodes[0].m_x.getY() << " | " << psb->m_nodes[0].m_v.getY() << std::endl;
    }

    for ( int i = 0; i < psb->m_nodes.size(); ++i) {
        psb->m_nodes[i].m_x.setY(0);
    }

    psb->scale( btVector3(100, 100, 100) );

    dumpState( mesh, psb, "_plane" );

    dynamicsWorld->removeSoftBody(psb);
    delete psb;

    dynamicsWorld->removeRigidBody(plane);
    delete plane->getMotionState();
    delete plane;
    delete groundShape;

    delete dynamicsWorld;
    delete solver;
    delete collisionConfiguration;
    delete dispatcher;
    delete broadphase;

    return 0;
}

void dumpState( VC_MeshType::Pointer toUpdate, btSoftBody* body, std::string suffix ) {
    VC_MeshType::Pointer output = VC_MeshType::New();
    volcart::meshing::deepCopy(toUpdate, output);
    volcart::meshing::bullet2itk::bullet2itk(body, output);

    std::string path = "inter_" + VC_DATE_TIME() + suffix + ".obj";
    volcart::io::objWriter writer(path, output);
    writer.write();
}