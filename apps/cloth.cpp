//
// Created by Abigail Coleman 10/28/15
//

#include <iostream>
#include <math.h>

#include <opencv2/opencv.hpp>

#include <vtkSmoothPolyDataFilter.h>
#include <vtkDecimatePro.h>
#include "vtkMassProperties.h"

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "volumepkg.h"
#include "io/ply2itk.h"
#include "itk2vtk.h"
#include "io/objWriter.h"
#include "compositeTextureV2.h"
#include "ACVD.h"
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

    // declare pointer to new Mesh object
    VC_MeshType::Pointer  mesh = VC_MeshType::New();

    int meshWidth = -1;
    int meshHeight = -1;

    // try to convert the ply to an ITK mesh
    if (!volcart::io::ply2itkmesh(meshName, mesh, meshWidth, meshHeight)) {
        exit( -1 );
    };

    vtkPolyData* vtkMesh = vtkPolyData::New();
    volcart::meshing::itk2vtk(mesh, vtkMesh);

    vtkSmartPointer<vtkMassProperties> massProperties = vtkMassProperties::New();
    massProperties->AddInputData(vtkMesh);
    double area = massProperties->GetSurfaceArea();
    double reduction = std::abs( 1 - ((area / 1.5) / mesh->GetNumberOfCells()) );
    std::cerr << "reduction: " << reduction << std::endl;

    vtkDecimatePro* decimatePro = vtkDecimatePro::New();
    decimatePro->SetInputData(vtkMesh);
    decimatePro->SetTargetReduction(reduction);
    decimatePro->SplittingOff();

    vtkSmoothPolyDataFilter* smoother = vtkSmoothPolyDataFilter::New();
    smoother->SetInputConnection( decimatePro->GetOutputPort() );
    smoother->SetNumberOfIterations(3);
    smoother->SetRelaxationFactor(0.3);
    smoother->Update();

    VC_MeshType::Pointer decimated = VC_MeshType::New();
    volcart::meshing::vtk2itk(smoother->GetOutput(), decimated);

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
    btSoftBody* psb;
    volcart::meshing::itk2bullet::itk2bullet(decimated, dynamicsWorld->getWorldInfo(), &psb);

    psb->getWorldInfo()->m_gravity = dynamicsWorld->getGravity(); // Have to explicitly make softbody gravity match world gravity
    dynamicsWorld->addSoftBody(psb);
    dumpState( decimated, psb, "_0" );

    // Constraints for the mesh as a soft body
    // These needed to be tested to find optimal values.
    // Sets the mass of the whole soft body, true considers the faces along with the vertices
    // Note: Mass is in kilograms. If mass isn't high enough, nothing changes.
    printf("volcart::cloth::message: Setting mass\n");
    psb->setTotalMass( (int)(psb->m_nodes.size() * 0.001), true );

    psb->m_cfg.kDP = 0.1; // Damping coefficient of the soft body [0,1]
    psb->m_materials[0]->m_kLST = 1.0; // Linear stiffness coefficient [0,1]
    psb->m_materials[0]->m_kAST = 1.0; // Area/Angular stiffness coefficient [0,1]
    psb->m_materials[0]->m_kVST = 1.0; // Volume stiffness coefficient [0,1]

    // Find the position of the four corner nodes
    // Currently assumes that the first point has the same z-value as the rest of the starting chain
    // This needs work. A lot of work.
    int min_z = (int) std::floor(mesh->GetPoint(0)[2]);
    unsigned long chain_size = 1;
    double chain_length = 0;
    // Calculate chain size and chain length
    for(unsigned long i = 1; i < mesh->GetNumberOfPoints(); ++i) {
        if( mesh->GetPoint(i)[2] <= min_z ) {
            chain_length += mesh->GetPoint(i).EuclideanDistanceTo(mesh->GetPoint(i-1));
            ++chain_size;
        }
        else
            break;
    }
    VC_PointType tl = mesh->GetPoint(0);
    VC_PointType tr = mesh->GetPoint(chain_size - 1);
    VC_PointType bl = mesh->GetPoint(mesh->GetNumberOfPoints() - chain_size);
    VC_PointType br = mesh->GetPoint(mesh->GetNumberOfPoints() - 1);

    unsigned long tl_id, tr_id, bl_id, br_id;
    tl_id = tr_id = bl_id = br_id = 0;
    for ( auto pt = decimated->GetPoints()->Begin(); pt != decimated->GetPoints()->End(); ++pt ) {
        if ( tl.EuclideanDistanceTo(pt->Value()) < tl.EuclideanDistanceTo(decimated->GetPoint(tl_id)) ) tl_id = pt->Index();
        if ( tr.EuclideanDistanceTo(pt->Value()) < tr.EuclideanDistanceTo(decimated->GetPoint(tr_id)) ) tr_id = pt->Index();
        if ( bl.EuclideanDistanceTo(pt->Value()) < bl.EuclideanDistanceTo(decimated->GetPoint(bl_id)) ) bl_id = pt->Index();
        if ( br.EuclideanDistanceTo(pt->Value()) < br.EuclideanDistanceTo(decimated->GetPoint(br_id)) ) br_id = pt->Index();
    }

    // For debug. These values should match.
    if ( chain_size != meshWidth ) return EXIT_FAILURE;

    // Append rigid bodies to respective nodes of mesh
    // Assumes the chain length is constant throughout the mesh
    btSoftBody::Node* top_left     = &psb->m_nodes[tl_id];
    btSoftBody::Node* top_right    = &psb->m_nodes[tr_id];
    btSoftBody::Node* bottom_left  = &psb->m_nodes[bl_id];
    btSoftBody::Node* bottom_right = &psb->m_nodes[br_id];

    pinnedPoints.push_back(top_left);
    pinnedPoints.push_back(top_right);
    pinnedPoints.push_back(bottom_left);
    pinnedPoints.push_back(bottom_right);

    double pinMass = 10;
    psb->setMass(tl_id, pinMass);
    psb->setMass(tr_id, pinMass);
    psb->setMass(bl_id, pinMass);
    psb->setMass(br_id, pinMass);

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
    node_ptr = &psb->m_nodes[tl_id];
    n_target.t_pos = psb->m_nodes[tl_id].m_x;
    n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
    targetPoints.push_back(n_target);

    // top right corner
    node_ptr = &psb->m_nodes[tr_id];
    t_x = psb->m_nodes[tl_id].m_x.x() + width;
    t_y = psb->m_nodes[tl_id].m_x.y();
    t_z = psb->m_nodes[tl_id].m_x.z();
    n_target.t_pos = btVector3(t_x, t_y, t_z);
    n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
    targetPoints.push_back(n_target);

    // bottom left corner
    node_ptr = &psb->m_nodes[bl_id];
    t_x = psb->m_nodes[tl_id].m_x.x();
    t_y = psb->m_nodes[tl_id].m_x.y();
    t_z = psb->m_nodes[tl_id].m_x.z() + height;
    n_target.t_pos = btVector3(t_x, t_y, t_z);
    n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
    targetPoints.push_back(n_target);

    // bottom right corner
    node_ptr = &psb->m_nodes[br_id];
    t_x = psb->m_nodes[tl_id].m_x.x() + width;
    t_y = psb->m_nodes[tl_id].m_x.y();
    t_z = psb->m_nodes[tl_id].m_x.z() + height;
    n_target.t_pos = btVector3(t_x, t_y, t_z);
    n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
    targetPoints.push_back(n_target);

    // Middle of target rectangle
    t_x = psb->m_nodes[tl_id].m_x.x() + (width / 2);
    t_y = psb->m_nodes[tl_id].m_x.y();
    t_z = psb->m_nodes[tl_id].m_x.z() + (height / 2);
    middle = btVector3(t_x, t_y, t_z);

    // Planarize the corners
    printf("volcart::cloth::message: Planarizing corners\n");
    dynamicsWorld->setInternalTickCallback(planarizeCornersPreTickCallback, dynamicsWorld, true);
    int counter = 0;
    while ( counter < required_iterations ) {
        std::cerr << "volcart::cloth::message: Step " << counter+1 << "/" << required_iterations << "\r" << std::flush;
        dynamicsWorld->stepSimulation(1/60.f);
        psb->solveConstraints();
        ++counter;
    }
    std::cerr << std::endl;
    printf("Planarize steps: %d\n", counter);
    dumpState( decimated, psb, "_1");

    // Expand the corners
    printf("volcart::cloth::message: Expanding corners\n");
    counter = 0;
    required_iterations = required_iterations * 2;
    while ( (btAverageNormal(psb).absolute().getY() < 0.925 || counter < required_iterations) ) {
        std::cerr << "volcart::cloth::message: Step " << counter+1 << "\r" << std::flush;
        if ( counter % 2000 == 0 ) expandCorners( 10 + (counter / 2000) );
        dynamicsWorld->stepSimulation(1/60.f);
        psb->solveConstraints();
        ++counter;
    }
    std::cerr << std::endl;
    printf("Expansion steps: %d\n", counter);
    dumpState( decimated, psb, "_2" );

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
    printf("volcart::cloth::message: Relaxing corners\n");
    dynamicsWorld->setInternalTickCallback(emptyPreTickCallback, dynamicsWorld, true);
    required_iterations = required_iterations * 2;
    counter = 0;
    double test_area = btSurfaceArea(psb);
    while ( isnan(test_area) || test_area/surface_area > 1.05 || counter < required_iterations ) {
        std::cerr << "volcart::cloth::message: Step " << counter+1 << "\r" << std::flush;
        dynamicsWorld->stepSimulation(1/60.f);
        psb->solveConstraints();

        ++counter;
        if ( counter % 500 == 0 ) test_area = btSurfaceArea(psb); // recalc area every 500 iterations
    }
    std::cerr << std::endl;
    printf("Relaxation steps: %d\n", counter);
    dumpState( decimated, psb, "_3" );

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
            decimated->GetCell(f_id, c);
            double p_id = c->GetPointIdsContainer()[n_id];

            // Add the uv coordinates into our map at the point index specified
            uvMap.set(p_id, uv);

        }
    }

    // Convert soft body to itk mesh
    volcart::texturing::compositeTextureV2 result(decimated, vpkg, uvMap, 10, (int) aspect_width * 2, (int) aspect_height * 2);
    volcart::io::objWriter objwriter("cloth.obj", decimated, result.texture().uvMap(), result.texture().getImage(0));
    objwriter.write();

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
