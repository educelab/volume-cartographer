//
// Created by Seth Parker on 6/24/15.
//

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "shapes.h"
#include "volumepkg.h"

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
  vpkg.setActiveSegmentation( argv[2] );

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

  // Decimate using ACVD
  vtkPolyData* acvdMesh = vtkPolyData::New();
  volcart::meshing::ACVD(vtkMesh, acvdMesh, 2000, 0, 0, 200);

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
  int width = std::ceil( uvMap.ratio().width );
  int height = std::ceil( (double) width / uvMap.ratio().aspect );

  volcart::texturing::compositeTextureV2 compText(outputMesh, vpkg, uvMap, 5, width, height);

  // Setup rendering
  volcart::Rendering rendering;
  rendering.setTexture( compText.texture() );
  rendering.setMesh( outputMesh );

  volcart::io::objWriter mesh_writer;
  mesh_writer.setPath( "lscm.obj" );
  mesh_writer.setRendering( rendering );
  mesh_writer.write();


  return EXIT_SUCCESS;
}