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

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>

int main(int argc, char* argv[])
{
	if ( argc < 4 ) {
		std::cout << "Usage: vc_texture2 volpkg seg-id radius" << std::endl;
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
  double radius, minorRadius;
  radius = atof( argv[ 3 ] );
  if((minorRadius = radius / 3) < 1) minorRadius = 1;

  // declare pointer to new Mesh object
  VC_MeshType::Pointer  mesh = VC_MeshType::New();

  int meshWidth = -1;
  int meshHeight = -1;

  // try to convert the ply to an ITK mesh
  if (!volcart::io::ply2itkmesh(meshName, mesh, meshWidth, meshHeight)){
    exit( -1 );
  };

  // Matrix to store the output texture
  int textureW = meshWidth;
  int textureH = meshHeight;
  cv::Mat outputTexture = cv::Mat::zeros( textureH, textureW, CV_16UC1 );

  // pointID == point's position in 1D list of points
  // [meshX, meshY] == point's position if list was a 2D matrix
  // [u, v] == point's position in the output matrix
  unsigned long pointID, meshX, meshY;
  double u, v;
    
  // Load the slices from the volumepkg
  std::vector< cv::Mat > aImgVol;

  /*  This function is a hack to avoid a refactoring the texturing
      methods. See Issue #12 for more details. */
  // Setup
  int meshLowIndex = (int) mesh->GetPoint(0)[2];
  int meshHighIndex = meshLowIndex + meshHeight;
  int aNumSlices = vpkg.getNumberOfSlices();

  int bufferLowIndex = meshLowIndex - (int) radius;
  if (bufferLowIndex < 0) bufferLowIndex = 0;

  int bufferHighIndex = meshHighIndex + (int) radius;
  if (bufferHighIndex >= vpkg.getNumberOfSlices()) bufferHighIndex = vpkg.getNumberOfSlices();

  // Slices must be loaded into aImgVol at the correct index: slice 005 == aImgVol[5]
  // To avoid loading the whole volume, pad the beginning indices with 1x1 null mats
  cv::Mat nullMat = cv::Mat::zeros(1, 1, CV_16U);
  for ( int i = 0; i < bufferLowIndex; ++i ) {
    std::cout << "\rLoading null buffer slices: " << i + 1 << "/" << bufferLowIndex << std::flush;
    aImgVol.push_back( nullMat.clone() );
  }
  std::cout << std::endl;

  // Load the actual volume into a tempVol with a buffer of nRadius
  for ( int i = bufferLowIndex; i < bufferHighIndex; ++i ) {
  std::cout << "\rLoading real slices: " << i - bufferLowIndex + 1 << "/" << bufferHighIndex - bufferLowIndex << std::flush;
    aImgVol.push_back( vpkg.getSliceData( i ).clone() );
  }
  std::cout << std::endl;

  // Generate per point uv coordinates specifically for the OBJ
  VC_UVMap uvMap;
  std::cerr << "Calculating UV coordinates for final mesh..." << std::endl;
  for ( VC_PointsInMeshIterator point = mesh->GetPoints()->Begin(); point != mesh->GetPoints()->End(); ++point ) {
    pointID = point.Index();

    // Calculate the point's [meshX, meshY] position based on its pointID
    meshX = pointID % meshWidth;
    meshY = (pointID - meshX) / meshWidth;

    // Calculate the point's UV position
    u =  (double) meshX / (double) meshWidth;
    v =  (double) meshY / (double) meshHeight;

    // OBJ UV coordinates start in the bottom-left of an image, but we're calculating them from the top-left
    // Subtract v from 1.0 to account for this
    cv::Vec2d uv( u, 1.0 - v );

    // Add the uv coordinates into our map at the point index specified
    uvMap.insert( {pointID, uv} );
  }

	// convert itk mesh to bullet mesh (vertice and triangle arrays)
  int NUM_OF_POINTS = mesh->GetNumberOfPoints();
  btScalar bulletPoints[NUM_OF_POINTS*3];
  int NUM_OF_CELLS = mesh->GetNumberOfCells();
  int bulletFaces[NUM_OF_CELLS][3];
  volcart::meshing::itk2bullet::itk2bullet(mesh, bulletPoints, bulletFaces);
  // for(int i = 0; i < mesh->GetNumberOfPoints(); i += 3) {
  //   std::cout << bulletPoints[i] << ", " << bulletPoints[i+1] << ", " << bulletPoints[i+2] << std::endl;
  // }
  // for(int i = 0; i < mesh->GetNumberOfCells(); ++i) {
  //   std::cout << bulletFaces[i][0] << ", " << bulletFaces[i][1] << ", " << bulletFaces[i][2] << std::endl;
  // };
  
  std::cout << "Bullet Mesh...........CHECK" << std::endl;
  // Create Dynamic world for bullet cloth simulation
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

  btSoftRigidDynamicsWorld* dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

  dynamicsWorld->setGravity(btVector3(0, -10, 0));

  // btSoftBody*	psb = btSoftBodyHelpers::CreateFromTriMesh(*(psb->getWorldInfo()), bulletPoints,
		// 																										 &bulletFaces[0][0],
		// 																									 	 NUM_OF_CELLS);

  btSoftBody*	psb = btSoftBodyHelpers::CreateFromTriMesh(dynamicsWorld->getWorldInfo(), bulletPoints,
																												 &bulletFaces[0][0],
																											 	 NUM_OF_CELLS);

  std::cout << "Bullet Soft Body...........CHECK" << std::endl;

  dynamicsWorld->addSoftBody(psb);

  std::cout << "Bullet Soft Body Added to World...........CHECK" << std::endl;

  // step simulation
  for (int i = 0; i < 300; i++) {
    dynamicsWorld->stepSimulation(1 / 60.f, 10);

    btTransform trans;
    trans = psb->getWorldTransform();

    std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;
  }

  std::cout << "Simulation...........CHECK" << std::endl;

  // bullet clean up
  dynamicsWorld->removeSoftBody(psb);
  // delete psb->getMotionState();
  delete psb;
  delete dynamicsWorld;
  delete solver;
  delete collisionConfiguration;
  delete dispatcher;
  delete broadphase;

	return 0;
} // end main