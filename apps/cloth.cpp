//
// Created by Abigail Coleman 10/28/15
//

#include <iostream>
#include <math.h>

#include <curses.h>

#include <opencv2/opencv.hpp>

#include "volumepkg.h"
#include "vc_defines.h"
#include "vc_datatypes.h"
#include "io/ply2itk.h"
#include "io/objWriter.h"
#include "compositeTexture.h"
#include "deepCopy.h"

// bullet converter
#include "itk2bullet.h"
#include <LinearMath/btVector3.h>

struct NodeTarget {
    btVector3 t_pos;
    btScalar  t_stepsize;
};

btVector3 middle(0,0,0);

btVector3 btAverageNormal( btSoftBody* body );
btScalar btAverageVelocity( btSoftBody* body );
double btSurfaceArea( btSoftBody* body );
static void planarizeCornersPreTickCallback(btDynamicsWorld *world, btScalar timeStep);
static void emptyPreTickCallback(btDynamicsWorld *world, btScalar timeStep);
void expandCorners(float magnitude);
void dumpState( VC_MeshType::Pointer toUpdate, btSoftBody* body, std::string suffix = "" );
std::vector<btSoftBody::Node*> pinnedPoints;
std::vector<NodeTarget> targetPoints;

int main(int argc, char* argv[]) {
    if ( argc < 3 ) {
        std::cout << "Usage: vc_texture2 volpkg seg-id iterations" << std::endl;
        return EXIT_FAILURE;
    }

    // for curses input
    initscr();
    noecho();
    clear();
    cbreak();
    nodelay(stdscr, TRUE);
    int ch;
    bool breakloop = false;
    printw("vc_cloth sim\n");

    VolumePkg vpkg = VolumePkg( argv[ 1 ] );
    std::string segID = argv[ 2 ];
    if (segID == "") {
    printw("ERROR: Incorrect/missing segmentation ID!\n");
    exit(EXIT_FAILURE);
    }
    if ( vpkg.getVersion() < 2.0) {
        printw("ERROR: Volume package is version %d but this program requires a version >= 2.0.\n", vpkg.getVersion() );
        exit(EXIT_FAILURE);
    }
    vpkg.setActiveSegmentation(segID);
    std::string meshName = vpkg.getMeshPath();

    int64_t NUM_OF_ITERATIONS = atoi( argv[ 3 ] );

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
    printw("volcart::cloth::message: Converting mesh to softBody\n");
    btSoftBody* psb;
    volcart::meshing::itk2bullet::itk2bullet(mesh, dynamicsWorld->getWorldInfo(), &psb);

    psb->getWorldInfo()->m_gravity = dynamicsWorld->getGravity(); // Have to explicitly make softbody gravity match world gravity
    dynamicsWorld->addSoftBody(psb);

    // Constraints for the mesh as a soft body
    // These needed to be tested to find optimal values.
    // Sets the mass of the whole soft body, true considers the faces along with the vertices
    // Note: Mass is in kilograms. If mass isn't high enough, nothing changes.
    printw("volcart::cloth::message: Setting mass\n");
    psb->setTotalMass( (int)(psb->m_nodes.size() * 0.001), true );

    psb->m_cfg.kDP = 0.1; // Damping coefficient of the soft body [0,1]
    psb->m_materials[0]->m_kLST = 1.0; // Linear stiffness coefficient [0,1]
    psb->m_materials[0]->m_kAST = 1.0; // Area/Angular stiffness coefficient [0,1]
    psb->m_materials[0]->m_kVST = 1.0; // Volume stiffness coefficient [0,1]

    // Find the position of the four corner nodes
    // Currently assumes that the first point has the same z-value as the rest of the starting chain
    int min_z = (int) std::floor(mesh->GetPoint(0)[2]);
    int chain_size = 1;
    double chain_length = 0;
    // Calculate chain size and chain length
    for(int i = 1; i < psb->m_nodes.size(); ++i) {
        if( (int)psb->m_nodes[i].m_x.z() <= min_z ) {
            chain_length += psb->m_nodes[i].m_x.distance(psb->m_nodes[i-1].m_x);
            ++chain_size;
        }
        else
            break;
    }

    // For debug. These values should match.
    if ( chain_size != meshWidth ) return EXIT_FAILURE;

    // Append rigid bodies to respective nodes of mesh
    // Assumes the chain length is constant throughout the mesh
    btSoftBody::Node* top_left     = &psb->m_nodes[0];
    btSoftBody::Node* top_right    = &psb->m_nodes[chain_size - 1];
    btSoftBody::Node* bottom_left  = &psb->m_nodes[psb->m_nodes.size() - chain_size];
    btSoftBody::Node* bottom_right = &psb->m_nodes[psb->m_nodes.size() - 1];

    pinnedPoints.push_back(top_left);
    pinnedPoints.push_back(top_right);
    pinnedPoints.push_back(bottom_left);
    pinnedPoints.push_back(bottom_right);

    double pinMass = 10;
    psb->setMass(0, pinMass);
    psb->setMass(chain_size - 1, pinMass);
    psb->setMass(psb->m_nodes.size() - chain_size, pinMass);
    psb->setMass(psb->m_nodes.size() - 1, pinMass);

    // Calculate the surface area of the mesh
    double surface_area = btSurfaceArea(psb);
    int dir = ( top_left->m_x.getX() < top_right->m_x.getX() ) ? 1 : -1;
    double width = chain_length * dir;
    double height = surface_area / chain_length;
    int required_iterations = NUM_OF_ITERATIONS; // Minimum iterations to reach target

    // Create target positions with step size for our four corners
    // NOTE: Must be created in the same order that the rigid bodies were put into pinnedPoints
    NodeTarget n_target;
    btScalar t_x, t_y, t_z;
    btSoftBody::Node* node_ptr;

    // top left corner
    node_ptr = &psb->m_nodes[0];
    n_target.t_pos = psb->m_nodes[0].m_x;
    n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
    targetPoints.push_back(n_target);

    // top right corner
    node_ptr = &psb->m_nodes[chain_size - 1];
    t_x = psb->m_nodes[0].m_x.x() + width;
    t_y = psb->m_nodes[0].m_x.y();
    t_z = psb->m_nodes[0].m_x.z();
    n_target.t_pos = btVector3(t_x, t_y, t_z);
    n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
    targetPoints.push_back(n_target);

    // bottom left corner
    node_ptr = &psb->m_nodes[psb->m_nodes.size() - chain_size];
    t_x = psb->m_nodes[0].m_x.x();
    t_y = psb->m_nodes[0].m_x.y();
    t_z = psb->m_nodes[0].m_x.z() + height;
    n_target.t_pos = btVector3(t_x, t_y, t_z);
    n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
    targetPoints.push_back(n_target);

    // bottom right corner
    node_ptr = &psb->m_nodes[psb->m_nodes.size() - 1];
    t_x = psb->m_nodes[0].m_x.x() + width;
    t_y = psb->m_nodes[0].m_x.y();
    t_z = psb->m_nodes[0].m_x.z() + height;
    n_target.t_pos = btVector3(t_x, t_y, t_z);
    n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
    targetPoints.push_back(n_target);

    // Middle of target rectangle
    t_x = psb->m_nodes[0].m_x.x() + (width / 2);
    t_y = psb->m_nodes[0].m_x.y();
    t_z = psb->m_nodes[0].m_x.z() + (height / 2);
    middle = btVector3(t_x, t_y, t_z);

    // Planarize the corners
    printw("volcart::cloth::message: Planarizing corners\n");
    dynamicsWorld->setInternalTickCallback(planarizeCornersPreTickCallback, dynamicsWorld, true);
    int counter = 0;
    while ( counter < required_iterations && !breakloop) {
        if ( (ch = getch()) != ERR ) {
            switch(ch) {
                case 's':
                    dumpState( mesh, psb );
                    break;
                case 'b':
                    breakloop = true;
                    break;
                case 'q':
                    endwin();
                    return 5;
            }
        }
        printw("volcart::cloth::message: Step %d/%d\r", counter+1, required_iterations);
        dynamicsWorld->stepSimulation(1/60.f);
        psb->solveConstraints();
        ++counter;
    }
    printw("\n");
    printw("Planarize steps: %d\n", counter);
    dumpState( mesh, psb, "_1");

    // Expand the corners
    printw("volcart::cloth::message: Expanding corners\n");
    counter = 0;
    breakloop = false;
    required_iterations = required_iterations * 2;
    while ( (btAverageNormal(psb).absolute().getY() < 0.99 || counter < required_iterations) && !breakloop ) {
        if ( (ch = getch()) != ERR ) {
            switch (ch) {
                case 's':
                    dumpState(mesh, psb);
                    break;
                case 'b':
                    breakloop = true;
                    break;
                case 'q':
                    endwin();
                    return 5;
            }
        }
        printw("volcart::cloth::message: Step %d\r", counter+1);
        if ( counter % 2000 == 0 ) expandCorners( 10 + (counter / 2000) );
        dynamicsWorld->stepSimulation(1/60.f);
        psb->solveConstraints();
        ++counter;
    }
    printw("\n");
    printw("Expansion steps: %d\n", counter);
    dumpState( mesh, psb, "_2" );

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
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, plane_y, 0), 1);
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody* plane = new btRigidBody(groundRigidBodyCI);
    dynamicsWorld->addRigidBody(plane);

    // Set the gravity so the mesh will be pushed onto the plane
    dynamicsWorld->setGravity(btVector3(0, -15, 0));
    psb->getWorldInfo()->m_gravity = dynamicsWorld->getGravity(); // Have to explicitly make softbody gravity match world gravity

    // set the friction of the plane and the mesh s.t. the mesh can easily flatten upon collision
    plane->setFriction(0.01); // (0-1] Default: 0.5
    psb->m_cfg.kDF = 0.01; // Dynamic friction coefficient (0-1] Default: 0.2

    // Let it settle
    printw("volcart::cloth::message: Relaxing corners\n");
    dynamicsWorld->setInternalTickCallback(emptyPreTickCallback, dynamicsWorld, true);
    required_iterations = required_iterations * 2;
    counter = 0;
    breakloop = false;
    double test_area = btSurfaceArea(psb);
    while ( (isnan(test_area) || test_area/surface_area > 1.05 || counter < required_iterations) && !breakloop ) {
        if ( (ch = getch()) != ERR ) {
            switch (ch) {
                case 's':
                    dumpState(mesh, psb);
                    break;
                case 'b':
                    breakloop = true;
                    break;
                case 'q':
                    endwin();
                    return 5;
            }
        }
        printw("volcart::cloth::message: Step %d\r", counter+1);
        dynamicsWorld->stepSimulation(1/60.f);
        psb->solveConstraints();

        ++counter;
        if ( counter % 500 == 0 ) test_area = btSurfaceArea(psb); // recalc area every 500 iterations
    }
    printw("\n");
    printw("Relaxation steps: %d\n", counter);
    printw("\n\n\n");

    // UV map setup
    double min_u = psb->m_nodes[0].m_x.x();
    double min_v = psb->m_nodes[0].m_x.z();
    double max_u = psb->m_nodes[0].m_x.x();
    double max_v = psb->m_nodes[0].m_x.z();
    for (size_t n_id = 0; n_id < psb->m_nodes.size(); ++n_id) {
        double _x = psb->m_nodes[n_id].m_x.x();
        double _z = psb->m_nodes[n_id].m_x.z();
        if ( _x < min_u && _x >= 0 && _z >= 0 ) min_u = psb->m_nodes[n_id].m_x.x();
        if ( _z < min_v && _z >= 0) min_v = psb->m_nodes[n_id].m_x.z();
        if ( _x > max_u && _z >= 0) max_u = psb->m_nodes[n_id].m_x.x();
        if ( _z > max_v && _z >= 0) max_v = psb->m_nodes[n_id].m_x.z();
    }

    // Round so that we have integer bounds
    min_u = std::floor(min_u);
    min_v = std::floor(min_v);
    max_u = std::ceil(max_u);
    max_v = std::ceil(max_v);

    double aspect_width = max_u - min_u;
    double aspect_height = max_v - min_v;
    double aspect = aspect_width / aspect_height;
    volcart::UVMap uvMap;
    uvMap.ratio(aspect_width, aspect_height);

    // Calculate uv coordinates
    double u, v;
    for (size_t f_id = 0; f_id < psb->m_faces.size(); ++f_id) {

        for(size_t n_id = 0; n_id < 3; ++n_id) {

            u = (psb->m_faces[f_id].m_n[n_id]->m_x.x() - min_u) / (max_u - min_u);
            v = (psb->m_faces[f_id].m_n[n_id]->m_x.z() - min_v) / (max_v - min_v);
            cv::Vec2d uv( u, v );

            // btSoftBody faces hold pointers to specific nodes, but we need the point id
            // Lookup the point ID of this node in the original ITK mesh
            VC_CellType::CellAutoPointer c;
            mesh->GetCell(f_id, c);
            double p_id = c->GetPointIdsContainer()[n_id];

            // Add the uv coordinates into our map at the point index specified
            uvMap.set(p_id, uv);

        }
    }

    // Convert soft body to itk mesh
    printw("volcart::cloth::message: Updating mesh\n");
    VC_MeshType::Pointer flatMesh = VC_MeshType::New();
    volcart::meshing::deepCopy(mesh, flatMesh);
    volcart::meshing::bullet2itk::bullet2itk(psb, flatMesh);

    volcart::texturing::compositeTexture result(mesh, vpkg, meshWidth, meshHeight, 10, VC_Composite_Option::Maximum, VC_Direction_Option::Bidirectional);
    volcart::Texture newTexture = result.texture();
    volcart::io::objWriter objwriter("cloth.obj", flatMesh, newTexture.uvMap(), newTexture.getImage(0));
    objwriter.write();

    volcart::texturing::compositeTexture flat(mesh, vpkg, uvMap, 10, VC_Composite_Option::Maximum, VC_Direction_Option::Bidirectional);
    cv::imwrite("new_uvmap.png", flat.texture().getImage(0) );

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

    printw("\n\nPress q to quit\n");
    breakloop = false;
    while (!breakloop) {
        if ( (ch = getch()) != ERR ) {
            switch (ch) {
                case 'q':
                    breakloop = true;
                    break;
            }
        }
    }
    endwin();
    return 0;

} // end main

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

void planarizeCornersPreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
    // Iterate over rigid bodies and move them towards their targets
    for( size_t p_id = 0; p_id < pinnedPoints.size(); ++p_id ) {
        if ( pinnedPoints[p_id]->m_x == targetPoints[p_id].t_pos ) continue;
        btVector3 delta = (targetPoints[p_id].t_pos - pinnedPoints[p_id]->m_x).normalized() * targetPoints[p_id].t_stepsize;
        pinnedPoints[p_id]->m_v += delta/timeStep;
    }
};

void emptyPreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
    // This call back is used to disable other callbacks
    // Particularly used for relaxing the four corners
};

void expandCorners(float magnitude) {
    btScalar stepSize = 1;
    btScalar _magnitude = magnitude;
    for( size_t p_id = 0; p_id < pinnedPoints.size(); ++p_id ) {
        targetPoints[p_id].t_pos += (targetPoints[p_id].t_pos - middle).normalized() * _magnitude;
        targetPoints[p_id].t_stepsize = stepSize;
    }
}

void dumpState( VC_MeshType::Pointer toUpdate, btSoftBody* body, std::string suffix ) {
    VC_MeshType::Pointer output = VC_MeshType::New();
    volcart::meshing::deepCopy(toUpdate, output);
    volcart::meshing::bullet2itk::bullet2itk(body, output);

    std::string path = "inter_" + VC_DATE_TIME() + suffix + ".obj";
    volcart::io::objWriter writer(path, output);
    writer.write();
}
