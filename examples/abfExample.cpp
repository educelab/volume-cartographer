//
// Created by Seth Parker on 6/24/15.
//

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "volumepkg.h"

#include "io/plyWriter.h"
#include "io/ply2itk.h"
#include "itk2vtk.h"

#include <itkSmoothingQuadEdgeMeshFilter.h>
#include <itkQuadEdgeMeshDecimationCriteria.h>
#include <itkQuadricDecimationQuadEdgeMeshFilter.h>

#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkCleanPolyData.h>
#include <vtkSmoothPolyDataFilter.h>

#include "abf.h"
#include "meshMath.h"
#include "ACVD.h"
#include "compositeTextureV2.h"

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

  // Calculate sampling density
  double voxelsize = vpkg.getVoxelSize();
  double sa = volcart::meshMath::SurfaceArea( input ) * (voxelsize * voxelsize) * (0.001 * 0.001); // convert vx^2 -> mm^2;
  double densityFactor = 50;
  uint16_t numberOfVertices = std::round(densityFactor * sa);

  // Convert to quad edge mesh and smooth the thing
  volcart::QuadMesh::Pointer qeRaw = volcart::QuadMesh::New();
  volcart::meshing::itk2itkQE( input, qeRaw );

  typedef itk::SmoothingQuadEdgeMeshFilter<volcart::QuadMesh, volcart::QuadMesh> QuadSmoother;
  itk::OnesMatrixCoefficients< volcart::QuadMesh > coeff0;
  QuadSmoother::Pointer smoother = QuadSmoother::New();
  smoother->SetInput( qeRaw );
  smoother->SetNumberOfIterations(3);
  smoother->SetCoefficientsMethod( &coeff0 );
  smoother->Update();

//  typedef itk::NumberOfPointsCriterion< volcart::QuadMesh > CriterionType;
//  typedef itk::QuadricDecimationQuadEdgeMeshFilter< volcart::QuadMesh, volcart::QuadMesh, CriterionType> DecimationType;
//  CriterionType::Pointer criterion = CriterionType::New();
//  DecimationType::Pointer decimate = DecimationType::New();
//  criterion->SetTopologicalChange(false);
//  criterion->SetNumberOfElements(numberOfVertices*4);
//
//  decimate->SetInput(smoother->GetOutput());
//  decimate->SetCriterion(criterion);
//  decimate->Update();
//
//  std::cout << decimate->GetOutput()->GetNumberOfCells() << " || " << decimate->GetOutput()->GetNumberOfPoints() << std::endl;

  VC_MeshType::Pointer smoothed = VC_MeshType::New();
  volcart::meshing::itkQE2itk( smoother->GetOutput(), smoothed );

//  // Convert to polydata
//  vtkPolyData* vtkMesh = vtkPolyData::New();
//  volcart::meshing::itk2vtk(smoothed, vtkMesh);
//
//  // Decimate using ACVD
//  vtkPolyData* acvdMesh = vtkPolyData::New();
//  volcart::meshing::ACVD(vtkMesh, acvdMesh, numberOfVertices);
//
//  // Merge Duplicates
//  // Note: This merging has to be the last in the process chain for some really weird reason. - SP
//  vtkSmartPointer<vtkCleanPolyData> Cleaner = vtkCleanPolyData::New();
//  Cleaner->SetInputData( acvdMesh );
//  Cleaner->ToleranceIsAbsoluteOn();
//  Cleaner->Update();
//
//  VC_MeshType::Pointer itkACVD = VC_MeshType::New();
//  volcart::meshing::vtk2itk( Cleaner->GetOutput(), itkACVD );

  volcart::io::objWriter writer( "decimate.obj", smoothed );
  writer.write();

  // ABF flattening
  volcart::texturing::abf abf(smoothed);
  abf.compute();

  VC_MeshType::Pointer abfMesh = abf.getMesh();
  writer.setMesh(abfMesh);
  writer.setPath("decimate_abf.obj");
  writer.write();

//  // Convert soft body to itk mesh
//  volcart::texturing::compositeTextureV2 result(inputMesh, vpkg, uvMap, 7, (int) aspect_width, (int) aspect_height);
//  volcart::io::objWriter objwriter("cloth.obj", inputMesh, result.texture().uvMap(), result.texture().getImage(0));
//  objwriter.write();


  return EXIT_SUCCESS;
}