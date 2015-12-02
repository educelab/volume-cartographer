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

int main(int argc, char* argv[])
{
	if ( argc < 5 ) {
		std::cout << "Usage: vc_texture2 volpkg seg-id radius iterations" << std::endl;
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

  int NUM_OF_ITERATIONS = atoi( argv[ 4 ] );

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

	// convert itk mesh to bullet mesh (vertice and triangle arrays)
  int NUM_OF_POINTS = mesh->GetNumberOfPoints();
  btScalar bulletPoints[NUM_OF_POINTS*3];
  int NUM_OF_CELLS = mesh->GetNumberOfCells();
  int bulletFaces[NUM_OF_CELLS][3];
  volcart::meshing::itk2bullet::itk2bullet(mesh, bulletPoints, bulletFaces);

  // DEBUGGING: Check if itk2bullet function returned correct arrays
  // for(int i = 0; i < mesh->GetNumberOfPoints(); i += 3) {
  //   std::cout << bulletPoints[i] << ", " << bulletPoints[i+1] << ", " << bulletPoints[i+2] << std::endl;
  // }
  // for(int i = 0; i < mesh->GetNumberOfCells(); ++i) {
  //   std::cout << bulletFaces[i][0] << ", " << bulletFaces[i][1] << ", " << bulletFaces[i][2] << std::endl;
  // };
  
  // Create Dynamic world for bullet cloth simulation
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  btDefaultCollisionConfiguration* collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

  btSoftBodySolver* softBodySolver = new btDefaultSoftBodySolver();

  btSoftRigidDynamicsWorld* dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher,
  																																							 broadphase, 
  																																							 solver, 
  																																							 collisionConfiguration, 
  																																							 softBodySolver);

  dynamicsWorld->setGravity(btVector3(0, -10, 0));

  btSoftBody*	psb = btSoftBodyHelpers::CreateFromTriMesh(dynamicsWorld->getWorldInfo(), bulletPoints,
																												 &bulletFaces[0][0],
																											 	 NUM_OF_CELLS);

  dynamicsWorld->addSoftBody(psb);

  // Constraints for the mesh as a soft body
  // These needed to be tested to find optimal values
  // sets the mass of the whole soft body, true considers the faces along with the vertices
  psb->setTotalMass(100, true);\
  // set the damping coefficient of the soft body [0,1]
  psb->m_cfg.kDP = 0.5;

  // Set the top row of vertices such that they wont move/fall
  for(int i = 0; i < NUM_OF_POINTS; ++i) {
  	if( (int)psb->m_nodes[i].m_x.z() <= 2) {
  		psb->setMass(i, 0);
  	}
  }

  // rotate mesh 90 degrees around the y axis
  psb->rotate(btQuaternion(0,SIMD_PI/2,0));

  // step simulation
  for (int i = 0; i < NUM_OF_ITERATIONS; i++) {
    dynamicsWorld->stepSimulation(1/ 60.f);
    psb->solveConstraints();

    std::cout << "Cloth simulation step: " << i << "/" << NUM_OF_ITERATIONS << "\r" << std::flush;

  //   //check if mesh is moving/falling
  //   // std::cout << "Cloth coordinate: " << psb->m_nodes[10000].m_x.x() << ", " << psb->m_nodes[10000].m_x.y() << ", " << psb->m_nodes[10000].m_x.z() << std::endl;
  }

  std::cout << std::endl;

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