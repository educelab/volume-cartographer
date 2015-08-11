// main.cpp
// Abigail Coleman Feb. 2015

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "volumepkg.h"
#include "vc_defines.h"
#include "io/ply2itk.h"
#include "itk2vtk.h"

#include "texturingUtils.h"
#include <itkRGBPixel.h>
#include "UPointMapping.h"

#include <vtkSmartPointer.h>
#include <vtkOBBTree.h>
#include <vtkPolyDataNormals.h>

int main(int argc, char* argv[]) {
  if ( argc < 7 ) {
    std::cout << "Usage: vc_texture2 volpkg seg-id radius texture-method sample-direction tracing-direction number-of-sections" << std::endl;
    std::cout << "Texture methods: " << std::endl;
    std::cout << "      0 = Intersection" << std::endl;
    std::cout << "      1 = Non-Maximum Suppression" << std::endl;
    std::cout << "      2 = Maximum" << std::endl;
    std::cout << "      3 = Minimum" << std::endl;
    std::cout << "      4 = Median w/ Averaging" << std::endl;
    std::cout << "      5 = Median" << std::endl;
    std::cout << "      6 = Mean" << std::endl;
    std::cout << std::endl;
    std::cout << "Sample Direction: " << std::endl;
    std::cout << "      0 = Omni" << std::endl;
    std::cout << "      1 = Positive" << std::endl;
    std::cout << "      2 = Negative" << std::endl;
    std::cout << "Ray Tracing Direction: " << std::endl;
    std::cout << "      0 = Clockwise" << std::endl;
    std::cout << "      1 = Counterclockwise" << std::endl;
    exit( -1 );
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

  int aFindBetterTextureMethod = atoi( argv[ 4 ] );
  EFilterOption aFilterOption = ( EFilterOption )aFindBetterTextureMethod;

  int aSampleDir = atoi( argv[ 5 ] ); // sampleDirection (0=omni, 1=positive, 2=negative)
  EDirectionOption aDirectionOption = ( EDirectionOption )aSampleDir;

  int aTraceDir = atoi( argv[ 6 ] ); //rayTracingDirection (0=clockwise, 1=counterclockwise)

  int sections = atoi( argv[7] ); // The number of sections (images) to generate

  // declare pointer to new Mesh object
  VC_MeshType::Pointer mesh = VC_MeshType::New();
  int meshWidth = -1;
  int meshHeight = -1;

  if (!volcart::io::ply2itkmesh(meshName, mesh, meshWidth, meshHeight)) {
    exit(-1);
  }

  // Convert the itk mesh to a vtk mesh
  vtkPolyData *vtkMesh = vtkPolyData::New();
  volcart::meshing::itk2vtk(mesh, vtkMesh);

  // Get the bounds of the mesh, needed for ray tracing
  double bounds[6];
  vtkMesh->GetBounds(bounds);

  // pointID == point's position in 1D list of points
  // [meshX, meshY] == point's position if list was a 2D matrix
  // [u, v] == point's position in the output matrix
  unsigned long pointID, meshX, meshY;
  double u, v;

  // Load the slices from the volumepkg
  std::vector< cv::Mat > aImgVol;

  // Get range (thickness) of material
  double range = vpkg.getMaterialThickness();
  range = range / vpkg.getVoxelSize(); // convert range from microns to pixels

  /*  This function is a hack to avoid a refactoring the texturing
      methods. See Issue #12 for more details. */
  // Setup
  int meshLowIndex = (int) mesh->GetPoint(0)[2];
  int meshHighIndex = meshLowIndex + meshHeight;
  int aNumSlices = vpkg.getNumberOfSlices();

  int bufferLowIndex = meshLowIndex - (int) radius;
  if (bufferLowIndex < 0) bufferLowIndex = 0;

  int bufferHighIndex = meshHighIndex + (int) radius;
  if (bufferHighIndex >= vpkg.getNumberOfSlices()) {
    bufferHighIndex = vpkg.getNumberOfSlices();
  }

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

  // Create point cloud to save new interpolated points into
  // mainly used for debugging
  pcl::PointCloud<pcl::PointNormal>::Ptr new_cloud ( new pcl::PointCloud<pcl::PointNormal> );

// Parameters used to create cylindrical ray tracing
#define PI_X2 (2 * 3.1415926)
#define OUT_X 2000
#define D_THETA (PI_X2 / OUT_X)

  // Essential data structure that will be saved as images after sectioning
  // Each matrix represent a section
  cv::Mat *outputTextures = new cv::Mat[ sections ];

  // Allocate each output texture to exact width and height
  for ( int i = 0; i < sections; ++i) {
    outputTextures[ i ] = cv::Mat::zeros( (int)(bounds[5] - bounds[4]), OUT_X, CV_16UC1 );
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Using vtk's OBBTree to test a ray's intersection with the faces/cells/triangles in the mesh

  // Generate normals for the cells of the mesh
  vtkSmartPointer<vtkPolyDataNormals> calcNormals = vtkSmartPointer<vtkPolyDataNormals>::New();
  calcNormals->SetInputData(vtkMesh);
  calcNormals->ComputeCellNormalsOn();
  calcNormals->Update();
  vtkMesh = calcNormals->GetOutput();

  // Creat vtk OBBTree
  vtkSmartPointer<vtkOBBTree> obbTree = vtkSmartPointer<vtkOBBTree>::New();
  obbTree->SetDataSet(vtkMesh);
  obbTree->BuildLocator();

  vtkSmartPointer<vtkPoints> intersectPoints = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkIdList> intersectCells = vtkSmartPointer<vtkIdList>::New();

  // For each slice/row generate rays and interpolate new points
  for (int z = (int)bounds[4]; z < (int)bounds[5]; ++z) {
    int counter = 0;

    int ycount = 0;
    double radian = 0;
    // generate rays cylindrically
    for (double r = 0; r < PI_X2; r += D_THETA, ycount++) {
      // Calculate the ray according to ray tracing direction
      if (aTraceDir == 1) {
        // counterclockwise
        radian -= D_THETA;
      } else {
        // clockwise (default)
        radian += D_THETA;
      }
      // Calculate the origin by averaging the bounds of each coordinate
      cv::Vec3f origin;
      origin(0) = (bounds[0] + bounds[1]) / 2;
      origin(1) = (bounds[2] + bounds[3]) / 2;
      origin(2) = z;
      // Calculate direction of ray according to current degree of rotation along the cylinder
      cv::Vec3f direction(cos(radian), sin(radian), 0);
      cv::normalize(direction);

      // Create a second point along the ray using the origin and direction
      cv::Vec3f end_point = origin + 1000*direction;
      //origin = origin - 1000*direction;

      double start[3] = {origin[0], origin[1], origin[2]};
      double end[3] = {end_point[0], end_point[1], end_point[2]};

      obbTree->IntersectWithLine(start, end, intersectPoints, intersectCells);

      std::cout << "NumPoints: " << intersectPoints->GetNumberOfPoints() <<  " | Z: " << z << " | R: " << r << std::endl;

      if ( intersectPoints->GetNumberOfPoints() > 0 ) {

        cv::Vec3f pt_pos;
        cv::Vec3f pt_norm;

        pt_pos[0] = intersectPoints->GetPoint(0)[0];
        pt_pos[1] = intersectPoints->GetPoint(0)[1];
        pt_pos[2] = intersectPoints->GetPoint(0)[2];
        pt_norm[0] = vtkMesh->GetCellData()->GetNormals()->GetTuple(intersectCells->GetId(0))[0];
        pt_norm[1] = vtkMesh->GetCellData()->GetNormals()->GetTuple(intersectCells->GetId(0))[1];
        pt_norm[2] = vtkMesh->GetCellData()->GetNormals()->GetTuple(intersectCells->GetId(0))[2];

        pcl::PointNormal pt;
        pt.x = pt_pos[0];
        pt.y = pt_pos[1];
        pt.z = pt_pos[2];
        pt.normal_x = pt_norm[0];
        pt.normal_y = pt_norm[1];
        pt.normal_z = pt_norm[2];

        new_cloud->push_back(pt);

        // pointer to array that holds intensity values calculated from texturing
        double *nData = new double[ sections ];

        Sectioning( sections,
                    range,
                    pt_pos,
                    pt_norm,
                    aImgVol,
                    radius,
                    aDirectionOption,
                    nData);

        // Fill in the output pixels with the values from Layering
        for ( int i = 0; i < sections; ++i ) {
          // cv::Mat.at uses (row, column)
          outputTextures[ i ].at < unsigned short > (z - bounds[4], ycount) = (unsigned short) nData[ i ];
        }
      }
    }
  }


  pcl::io::savePCDFileASCII("resampled.pcd", *new_cloud);
  // vpkg.saveTextureData(outputTexture);
  for ( int i = 0; i < sections; ++i ){
    vpkg.saveTextureData( outputTextures[ i ], std::to_string( i ) );
  }
  return 0;
} // main
