//
// Created by Seth Parker on 6/24/15.
//

#include "common/vc_defines.h"
#include "volumepkg/volumepkg.h"

#include "common/io/PLYReader.h"
#include "common/io/plyWriter.h"
#include "meshing/itk2vtk.h"

#include <itkQuadEdgeMeshDecimationCriteria.h>
#include <itkQuadricDecimationQuadEdgeMeshFilter.h>
#include <itkSmoothingQuadEdgeMeshFilter.h>

#include <vtkCleanPolyData.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkSmoothPolyDataFilter.h>

#include "common/io/objWriter.h"
#include "common/types/Rendering.h"
#include "common/util/meshMath.h"
#include "meshing/ACVD.h"
#include "texturing/AngleBasedFlattening.h"
#include "texturing/compositeTextureV2.h"

namespace fs = boost::filesystem;

int main(int argc, char* argv[])
{

    VolumePkg vpkg(argv[1]);
    vpkg.volume().setCacheMemoryInBytes(10000000000);
    vpkg.setActiveSegmentation(argv[2]);
    int radius = std::stoi(argv[3]);
    int type = std::stoi(argv[4]);

    // Read the mesh
    fs::path meshName = vpkg.getMeshPath();

    // declare pointer to new Mesh object
    auto input = volcart::ITKMesh::New();

    // try to convert the ply to an ITK mesh
    if (!volcart::io::PLYReader(meshName, input)) {
        exit(-1);
    };

    // Calculate sampling density
    double voxelsize = vpkg.getVoxelSize();
    double sa = volcart::meshMath::SurfaceArea(input) *
                (voxelsize * voxelsize) *
                (0.001 * 0.001);  // convert vx^2 -> mm^2;
    double densityFactor = 50;
    uint16_t numberOfVertices = std::round(densityFactor * sa);

    // Convert to quad edge mesh and smooth the thing
    volcart::QuadMesh::Pointer qeRaw = volcart::QuadMesh::New();
    volcart::meshing::itk2itkQE(input, qeRaw);

    typedef itk::SmoothingQuadEdgeMeshFilter<
        volcart::QuadMesh, volcart::QuadMesh>
        QuadSmoother;
    itk::OnesMatrixCoefficients<volcart::QuadMesh> coeff0;
    QuadSmoother::Pointer smoother = QuadSmoother::New();
    smoother->SetInput(qeRaw);
    smoother->SetNumberOfIterations(3);
    smoother->SetCoefficientsMethod(&coeff0);
    smoother->Update();

    auto smoothed = volcart::ITKMesh::New();
    volcart::meshing::itkQE2itk(smoother->GetOutput(), smoothed);

    // Convert to polydata
    vtkPolyData* vtkMesh = vtkPolyData::New();
    volcart::meshing::itk2vtk(input, vtkMesh);

    // Decimate using ACVD
    std::cout << "Resampling mesh..." << std::endl;
    vtkPolyData* acvdMesh = vtkPolyData::New();
    volcart::meshing::ACVD(vtkMesh, acvdMesh, numberOfVertices);

    // Merge Duplicates
    // Note: This merging has to be the last in the process chain for some
    // really weird reason. - SP
    vtkSmartPointer<vtkCleanPolyData> Cleaner = vtkCleanPolyData::New();
    Cleaner->SetInputData(acvdMesh);
    Cleaner->ToleranceIsAbsoluteOn();
    Cleaner->Update();

    auto itkACVD = volcart::ITKMesh::New();
    volcart::meshing::vtk2itk(Cleaner->GetOutput(), itkACVD);

    // ABF flattening
    std::cout << "Computing parameterization..." << std::endl;
    volcart::texturing::AngleBasedFlattening abf(itkACVD);
    // abf.setABFMaxIterations(5);
    abf.compute();

    // Get uv map
    volcart::UVMap uvMap = abf.getUVMap();
    int width = std::ceil(uvMap.ratio().width);
    int height = std::ceil((double)width / uvMap.ratio().aspect);

    std::cout << width << "x" << height << std::endl;

    volcart::texturing::compositeTextureV2 compText(
        itkACVD, vpkg, uvMap, radius, width, height,
        (volcart::CompositeOption)type);

    // Setup rendering
    volcart::Rendering rendering;
    rendering.setTexture(compText.texture());
    rendering.setMesh(itkACVD);

    volcart::io::objWriter mesh_writer;
    mesh_writer.setPath("textured-" + std::to_string(type) + ".obj");
    mesh_writer.setRendering(rendering);
    mesh_writer.write();

    return EXIT_SUCCESS;
}
