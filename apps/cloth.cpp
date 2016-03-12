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
#include "itkQuadEdgeMeshBoundaryEdgesMeshFunction.h"
#include "itkMeshFileReader.h"

// bullet converter
#include "itk2bullet.h"
#include <LinearMath/btVector3.h>
#include <itkOBJMeshIO.h>

struct NodeTarget {
    btVector3 t_pos;
    btScalar  t_stepsize;
};

struct PinnedPoint {
    btSoftBody::Node* node;
    NodeTarget        target;
};
std::vector< PinnedPoint > pinnedPoints;
btVector3 middle(0,0,0);

//btVector3 btAverageNormal( btSoftBody* body );
//btScalar btAverageVelocity( btSoftBody* body );
//double btSurfaceArea( btSoftBody* body );
//static void planarizeCornersPreTickCallback(btDynamicsWorld *world, btScalar timeStep);
//static void emptyPreTickCallback(btDynamicsWorld *world, btScalar timeStep);
//void expandCorners(float magnitude);
void dumpState( VC_MeshType::Pointer toUpdate, btSoftBody* body, std::string suffix = "" );

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

    dynamicsWorld->setGravity(btVector3(0, 0, 0));

    // convert itk mesh to bullet mesh (vertices and triangle arrays)
    btSoftBody* psb;
    volcart::meshing::itk2bullet::itk2bullet(mesh, dynamicsWorld->getWorldInfo(), &psb);

    psb->getWorldInfo()->m_gravity = dynamicsWorld->getGravity(); // Have to explicitly make softbody gravity match world gravity
    dynamicsWorld->addSoftBody(psb);
    //dumpState( mesh, psb, "_0" );

    // Constraints for the mesh as a soft body
    // These needed to be tested to find optimal values.
    // Sets the mass of the whole soft body, true considers the faces along with the vertices
    // Note: Mass is in kilograms. If mass isn't high enough, nothing changes.
    printf("volcart::cloth::message: Setting mass\n");
    psb->setTotalMass( (int)(psb->m_nodes.size() * 0.001), true );

    psb->m_cfg.kDP = 0.01; // Damping coefficient of the soft body [0,1]
    psb->m_materials[0]->m_kLST = 1.0; // Linear stiffness coefficient [0,1]
    psb->m_materials[0]->m_kAST = 1.0; // Area/Angular stiffness coefficient [0,1]
    psb->m_materials[0]->m_kVST = 1.0; // Volume stiffness coefficient [0,1]


    // Get QMesh
    typedef itk::MeshFileReader< VC_QuadMeshType >  QReaderType;
    QReaderType::Pointer qReader = QReaderType::New();
    qReader->SetFileName( "decim.obj" );
    qReader->Update();
    VC_QuadMeshType::Pointer QMesh = qReader->GetOutput();

    // Get the boundary points
    typedef itk::QuadEdgeMeshBoundaryEdgesMeshFunction< VC_QuadMeshType > BoundaryExtractorType;
    BoundaryExtractorType::Pointer extractor = BoundaryExtractorType::New();
    VC_QuadEdgeListPointer list = extractor->Evaluate( *QMesh );
    if( list->empty() )
    {
        std::cerr << "There is no border on this mesh" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "There are " << list->size() << " borders on this mesh" << std::endl;
    VC_QuadMeshIteratorGeom eIt = (*list->begin())->BeginGeomLnext();
    const VC_QuadMeshIteratorGeom eEnd = (*list->begin())->EndGeomLnext();
    VC_QuadMeshPointIdentifier start = eIt.Value()->GetOrigin();

    int iterations_this_phase = NUM_OF_ITERATIONS; // Minimum iterations to reach target
    while( eIt != eEnd ) {

        PinnedPoint pin;

        if( eIt == (*list->begin())->BeginGeomLnext() ) {
            pin.node = &psb->m_nodes[start];
            pin.target.t_pos = pin.node->m_x;
            pin.target.t_stepsize = 0;
        } else {

            // Set the node reference
            unsigned long origin_ID = eIt.Value()->GetOrigin();
            pin.node = &psb->m_nodes[origin_ID];

            // Get the previous point in the boundary
            auto prev = pinnedPoints.back();

            // Get the distance and direction along the XY component
            int dir = ( prev.node->m_x.getX() <= pin.node->m_x.getX() ) ? 1 : -1;
            cv::Vec2d A(prev.node->m_x.getX(), prev.node->m_x.getY());
            cv::Vec2d B(pin.node->m_x.getX(), pin.node->m_x.getY());
            btScalar distance = cv::norm( A, B );

            // Apply the distance to only the x component, relative to the previous point's target position
            // This new point keeps it's original Z position
            pin.target.t_pos.setX( prev.target.t_pos.getX() + (dir * distance) );
            pin.target.t_pos.setY( prev.target.t_pos.getY() );
            pin.target.t_pos.setZ( pin.node->m_x.getZ() );
            pin.target.t_stepsize = pin.node->m_x.distance(pin.target.t_pos) / iterations_this_phase;
        }

        pinnedPoints.push_back( pin );
        ++eIt;
    }

    VC_MeshType::Pointer outline = VC_MeshType::New();

    std::cout << pinnedPoints.size() << std::endl;
    VC_PointType p;
    VC_PixelType n;
    double min_u, max_u, min_v, max_v;
    unsigned long id = 0;
    for ( auto it = pinnedPoints.begin(); it != pinnedPoints.end(); ++it, ++id ) {

        p[0] = it->target.t_pos.getX();
        p[1] = it->target.t_pos.getY();
        p[2] = it->target.t_pos.getZ();
        n[0] = 0;
        n[1] = 0;
        n[2] = 0;

        outline->SetPoint(id, p);
        outline->SetPointData(id, n);

        // Bounding box
        double _x = it->target.t_pos.getX();
        double _z = it->target.t_pos.getZ();
        if ( it == pinnedPoints.begin() ) {
            min_u = _x;
            min_v = _z;
            max_u = _x;
            max_v = _z;
        } else {
            if ( _x < min_u ) min_u = _x;
            if ( _z < min_v ) min_v = _z;
            if ( _x > max_u ) max_u = _x;
            if ( _z > max_v ) max_v = _z;
        }
    }

    // Round so that we have integer bounds
    min_u = std::floor(min_u);
    min_v = std::floor(min_v);
    max_u = std::ceil(max_u);
    max_v = std::ceil(max_v);

    double aspect_width = max_u - min_u;
    double aspect_height = max_v - min_v;

    cv::Mat points = cv::Mat::zeros( aspect_height, aspect_width, CV_8UC3 );
    for ( auto it = pinnedPoints.begin(); it != pinnedPoints.end(); ++it, ++id ) {

        // starting point
        int start_x = cvRound(it->target.t_pos.getX()) - min_u;
        int start_y = cvRound(it->target.t_pos.getZ()) - min_v;

        // next point
        auto next = std::next(it);
        int end_x = cvRound(next->target.t_pos.getX()) - min_u;
        int end_y = cvRound(next->target.t_pos.getZ()) - min_v;

        cv::line(points, cvPoint(start_x, start_y), cvPoint(end_x, end_y), cvScalar(255,0,0));

    }

    cv::imwrite("outline.png", points);

    volcart::io::plyWriter writer;
    writer.setMesh(outline);
    writer.setPath("outline.ply");
    writer.write();

    return 0;

//    // Append rigid bodies to respective nodes of mesh
//    // Assumes the chain length is constant throughout the mesh
//    btSoftBody::Node* top_left     = &psb->m_nodes[tl_id];
//    btSoftBody::Node* top_right    = &psb->m_nodes[tr_id];
//    btSoftBody::Node* bottom_left  = &psb->m_nodes[bl_id];
//    btSoftBody::Node* bottom_right = &psb->m_nodes[br_id];
//
//    pinnedPoints.push_back(top_left);
//    pinnedPoints.push_back(top_right);
//    pinnedPoints.push_back(bottom_left);
//    pinnedPoints.push_back(bottom_right);
//
//    double pinMass = 10;
//    psb->setMass(tl_id, pinMass);
//    psb->setMass(tr_id, pinMass);
//    psb->setMass(bl_id, pinMass);
//    psb->setMass(br_id, pinMass);
//
//    // Calculate the surface area of the mesh
//    double surface_area = btSurfaceArea(psb);
//    int dir = ( top_left->m_x.getX() < top_right->m_x.getX() ) ? 1 : -1;
//    double width = chain_length * dir;
//    double height = bl[2] - tl[2];
//
//    // Create target positions with step size for our four corners
//    // NOTE: Must be created in the same order that the rigid bodies were put into pinnedPoints
//    NodeTarget n_target;
//    btScalar t_x, t_y, t_z;
//    btSoftBody::Node* node_ptr;
//
//    // top left corner
//    node_ptr = &psb->m_nodes[tl_id];
//    n_target.t_pos = psb->m_nodes[tl_id].m_x;
//    n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
//    targetPoints.push_back(n_target);
//
//    // top right corner
//    node_ptr = &psb->m_nodes[tr_id];
//    t_x = psb->m_nodes[tl_id].m_x.x() + width;
//    t_y = psb->m_nodes[tl_id].m_x.y();
//    t_z = psb->m_nodes[tl_id].m_x.z();
//    n_target.t_pos = btVector3(t_x, t_y, t_z);
//    n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
//    targetPoints.push_back(n_target);
//
//    // bottom left corner
//    node_ptr = &psb->m_nodes[bl_id];
//    t_x = psb->m_nodes[tl_id].m_x.x();
//    t_y = psb->m_nodes[tl_id].m_x.y();
//    t_z = psb->m_nodes[tl_id].m_x.z() + height;
//    n_target.t_pos = btVector3(t_x, t_y, t_z);
//    n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
//    targetPoints.push_back(n_target);
//
//    // bottom right corner
//    node_ptr = &psb->m_nodes[br_id];
//    t_x = psb->m_nodes[tl_id].m_x.x() + width;
//    t_y = psb->m_nodes[tl_id].m_x.y();
//    t_z = psb->m_nodes[tl_id].m_x.z() + height;
//    n_target.t_pos = btVector3(t_x, t_y, t_z);
//    n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
//    targetPoints.push_back(n_target);
//
//    // Middle of target rectangle
//    t_x = psb->m_nodes[tl_id].m_x.x() + (width / 2);
//    t_y = psb->m_nodes[tl_id].m_x.y();
//    t_z = psb->m_nodes[tl_id].m_x.z() + (height / 2);
//    middle = btVector3(t_x, t_y, t_z);
//
//    // Planarize the corners
//    printf("volcart::cloth::message: Planarizing corners\n");
//    dynamicsWorld->setInternalTickCallback(planarizeCornersPreTickCallback, dynamicsWorld, true);
//    int counter = 0;
//    while ( counter < required_iterations ) {
//        std::cerr << "volcart::cloth::message: Step " << counter+1 << "/" << required_iterations << "\r" << std::flush;
//        dynamicsWorld->stepSimulation(1/60.f);
//        psb->solveConstraints();
//        ++counter;
//    }
//    std::cerr << std::endl;
//    printf("Planarize steps: %d\n", counter);
//    dumpState( mesh, psb, "_1");
//
//    // Expand the corners
//    printf("volcart::cloth::message: Expanding corners\n");
//    counter = 0;
//    required_iterations = required_iterations * 2;
//    while ( (btAverageNormal(psb).absolute().getY() < 0.925 || counter < required_iterations) && counter < required_iterations*2 ) {
//        std::cerr << "volcart::cloth::message: Step " << counter+1 << "\r" << std::flush;
//        if ( counter % 2000 == 0 ) expandCorners( 10 + (counter / 2000) );
//        dynamicsWorld->stepSimulation(1/60.f);
//        psb->solveConstraints();
//        ++counter;
//    }
//    std::cerr << std::endl;
//    printf("Expansion steps: %d\n", counter);
//    dumpState( mesh, psb, "_2" );
//
//    // Add a collision plane to push the mesh onto
//    btScalar min_y = psb->m_nodes[0].m_x.y();
//    btScalar max_y = psb->m_nodes[0].m_x.y();
//    for (size_t n_id = 1; n_id < psb->m_nodes.size(); ++n_id) {
//        double _y = psb->m_nodes[n_id].m_x.y();
//        double _z = psb->m_nodes[n_id].m_x.z();
//        if ( _y < min_y && _z >= 0) min_y = psb->m_nodes[n_id].m_x.y();
//        if ( _y > max_y && _z >= 0) max_y = psb->m_nodes[n_id].m_x.y();
//    }
//
//    btScalar plane_y = min_y - 5;
//    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, plane_y, 0), 1);
//    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
//    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
//    btRigidBody* plane = new btRigidBody(groundRigidBodyCI);
//    dynamicsWorld->addRigidBody(plane);
//
//    // Set the gravity so the mesh will be pushed onto the plane
//    dynamicsWorld->setGravity(btVector3(0, -15, 0));
//    psb->getWorldInfo()->m_gravity = dynamicsWorld->getGravity(); // Have to explicitly make softbody gravity match world gravity
//
//    // set the friction of the plane and the mesh s.t. the mesh can easily flatten upon collision
//    plane->setFriction(0.01); // (0-1] Default: 0.5
//    psb->m_cfg.kDF = 0.01; // Dynamic friction coefficient (0-1] Default: 0.2
//    psb->m_cfg.kDP = 0.1; // Damping coefficient of the soft body [0,1]
//
//    // Let it settle
//    printf("volcart::cloth::message: Relaxing corners\n");
//    dynamicsWorld->setInternalTickCallback(emptyPreTickCallback, dynamicsWorld, true);
//    required_iterations = required_iterations * 2;
//    counter = 0;
//    double test_area = btSurfaceArea(psb);
//    while ( (isnan(test_area) || test_area/surface_area > 1.05 || counter < required_iterations) && counter < required_iterations*4 ) {
//        std::cerr << "volcart::cloth::message: Step " << counter+1 << "\r" << std::flush;
//        dynamicsWorld->stepSimulation(1/60.f);
//        psb->solveConstraints();
//
//        ++counter;
//        if ( counter % 500 == 0 ) test_area = btSurfaceArea(psb); // recalc area every 500 iterations
//    }
//    std::cerr << std::endl;
//    printf("Relaxation steps: %d\n", counter);
//    dumpState( mesh, psb, "_3" );
//
//    // bullet clean up
//    dynamicsWorld->removeRigidBody(plane);
//    delete plane->getMotionState();
//    delete plane;
//    delete groundShape;
//    dynamicsWorld->removeSoftBody(psb);
//    delete psb;
//    delete dynamicsWorld;
//    delete softBodySolver;
//    delete solver;
//    delete dispatcher;
//    delete collisionConfiguration;
//    delete broadphase;
//
//    return 0;

} // end main

//btVector3 btAverageNormal(btSoftBody* body) {
//    btVector3  avg_normal(0,0,0);
//    for ( size_t n_id = 0; n_id < body->m_faces.size(); ++n_id ) {
//        avg_normal += body->m_faces[n_id].m_normal;
//    }
//    avg_normal /= body->m_faces.size();
//    return avg_normal;
//};
//
//btScalar btAverageVelocity(btSoftBody* body) {
//    btScalar velocity = 0;
//    for ( size_t n_id = 0; n_id < body->m_nodes.size(); ++n_id ) {
//        velocity += body->m_nodes[n_id].m_v.length();
//    }
//    velocity /= body->m_nodes.size();
//    return velocity;
//}
//
//// Calculate the surface area of the mesh using Heron's formula
//// Let a,b,c be the lengths of the sides of a triangle and p the semiperimeter
//// p = (a +  b + c) / 2
//// area of triangle = sqrt( p * (p - a) * (p - b) * (p - c) )
//double btSurfaceArea( btSoftBody* body ) {
//    double surface_area = 0;
//    for(int i = 0; i < body->m_faces.size(); ++i) {
//        double a = 0, b = 0, c = 0, p = 0;
//        a = body->m_faces[i].m_n[0]->m_x.distance(body->m_faces[i].m_n[1]->m_x);
//        b = body->m_faces[i].m_n[0]->m_x.distance(body->m_faces[i].m_n[2]->m_x);
//        c = body->m_faces[i].m_n[1]->m_x.distance(body->m_faces[i].m_n[2]->m_x);
//
//        p = (a + b + c) / 2;
//
//        surface_area += sqrt( p * (p - a) * (p - b) * (p - c) );
//    }
//
//    return surface_area;
//}
//
//void planarizeCornersPreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
//    // Iterate over rigid bodies and move them towards their targets
//    for( size_t p_id = 0; p_id < pinnedPoints.size(); ++p_id ) {
//        if ( pinnedPoints[p_id]->m_x == targetPoints[p_id].t_pos ) continue;
//        btVector3 delta = (targetPoints[p_id].t_pos - pinnedPoints[p_id]->m_x).normalized() * targetPoints[p_id].t_stepsize;
//        pinnedPoints[p_id]->m_v += delta/timeStep;
//    }
//};
//
//void emptyPreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
//    // This call back is used to disable other callbacks
//    // Particularly used for relaxing the four corners
//};
//
//void expandCorners(float magnitude) {
//    btScalar stepSize = 1;
//    btScalar _magnitude = magnitude;
//    for( size_t p_id = 0; p_id < pinnedPoints.size(); ++p_id ) {
//        targetPoints[p_id].t_pos += (targetPoints[p_id].t_pos - middle).normalized() * _magnitude;
//        targetPoints[p_id].t_stepsize = stepSize;
//    }
//}
//
void dumpState( VC_MeshType::Pointer toUpdate, btSoftBody* body, std::string suffix ) {
    VC_MeshType::Pointer output = VC_MeshType::New();
    volcart::meshing::deepCopy(toUpdate, output);
    volcart::meshing::bullet2itk::bullet2itk(body, output);

    std::string path = "inter_" + VC_DATE_TIME() + suffix + ".obj";
    volcart::io::objWriter writer(path, output);
    writer.write();
}
