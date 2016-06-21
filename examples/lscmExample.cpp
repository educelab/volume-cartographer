//
// Created by Seth Parker on 6/24/15.
//

#include <algorithm>

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "shapes.h"
#include "volumepkg.h"
#include "meshMath.h"

#include "io/plyWriter.h"
#include "io/ply2itk.h"
#include "itk2vtk.h"
#include "ACVD.h"
#include <vtkCleanPolyData.h>
#include <vtkSmoothPolyDataFilter.h>

#include "lscm.h"
#include "compositeTextureV2.h"

#include <vtkPLYReader.h>

int main( int argc, char* argv[] ) {

  VolumePkg vpkg( argv[1] );
  vpkg.volume().setCacheMemoryInBytes(systemMemorySize());
  std::cout << "Cache size: " << vpkg.volume().getCacheCapacity() << std::endl;
  vpkg.setActiveSegmentation( argv[2] );
  int radius = std::stoi( argv[3] );
  int type = std::stoi( argv[4] );

  // Read the mesh
  std::string meshName = vpkg.getMeshPath();

  // declare pointer to new Mesh object
  VC_MeshType::Pointer  input = VC_MeshType::New();
  int meshWidth = -1;
  int meshHeight = -1;

  // try to convert the ply to an ITK mesh
  if (!volcart::io::ply2itkmesh(meshName, input, meshWidth, meshHeight)){
    exit( -1 );
  };

  vtkPolyData* vtkMesh = vtkPolyData::New();
  volcart::meshing::itk2vtk(input, vtkMesh);

  double voxelsize = vpkg.getVoxelSize();
  double sa = volcart::meshMath::SurfaceArea( input ) * (voxelsize * voxelsize) * (0.001 * 0.001); // convert vx^2 -> mm^2;
  double densityFactor = 50;
  uint16_t numberOfVertices = std::round(densityFactor * sa);

  // Decimate using ACVD
  vtkPolyData* acvdMesh = vtkPolyData::New();
  volcart::meshing::ACVD(vtkMesh, acvdMesh, numberOfVertices, 0, 0, 20);

  // Smooth points
  vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
  smoothFilter->SetInputData( acvdMesh );
  smoothFilter->SetNumberOfIterations(50);
  smoothFilter->SetRelaxationFactor(0.05);
  smoothFilter->FeatureEdgeSmoothingOn();
  smoothFilter->BoundarySmoothingOff();
  smoothFilter->Update();

  // Merge Duplicates
  // Note: This merging has to be the last in the process chain for some really weird reason. - SP
  vtkSmartPointer<vtkCleanPolyData> Cleaner = vtkCleanPolyData::New();
  Cleaner->SetInputConnection( smoothFilter->GetOutputPort() );
  Cleaner->ToleranceIsAbsoluteOn();
  Cleaner->Update();

  // Convert back to ITK mesh
  VC_MeshType::Pointer outputMesh = VC_MeshType::New();
  volcart::meshing::vtk2itk( Cleaner->GetOutput(), outputMesh);

  // Compute parameterization
  volcart::texturing::lscm lscm( outputMesh );
  lscm.compute();

  // Get uv map
  volcart::UVMap uvMap = lscm.getUVMap();
  int width = std::min( std::ceil( uvMap.ratio().width ), 1000.0 ) ;
  int height = std::ceil( (double) width / uvMap.ratio().aspect );

  std::cout << width << "x" << height << std::endl;

  volcart::texturing::compositeTextureV2 compText(outputMesh, vpkg, uvMap, radius, width, height, (VC_Composite_Option) type);

  // Setup rendering
  volcart::Rendering rendering;
  rendering.setTexture( compText.texture() );
  rendering.setMesh( outputMesh );

  volcart::io::objWriter mesh_writer;
  mesh_writer.setPath( "textured-" + std::to_string(type) + ".obj" );
  mesh_writer.setRendering( rendering );
  mesh_writer.write();


  return EXIT_SUCCESS;
}