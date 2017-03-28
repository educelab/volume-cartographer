//
// Created by Seth Parker on 6/24/15.
//

#include <itkQuadEdgeMeshDecimationCriteria.h>
#include <itkQuadricDecimationQuadEdgeMeshFilter.h>
#include <itkSmoothingQuadEdgeMeshFilter.h>
#include <vtkCleanPolyData.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkSmoothPolyDataFilter.h>

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYReader.hpp"
#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/types/Rendering.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/MeshMath.hpp"
#include "vc/core/vc_defines.hpp"
#include "vc/meshing/ACVD.hpp"
#include "vc/meshing/ITK2VTK.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"
#include "vc/texturing/CompositeTextureV2.hpp"

namespace fs = boost::filesystem;

int main(int /*argc*/, char* argv[])
{

    VolumePkg vpkg(argv[1]);
    vpkg.volume()->setCacheMemoryInBytes(10000000000);
    vpkg.setActiveSegmentation(argv[2]);
    int radius = std::stoi(argv[3]);
    auto type = static_cast<volcart::CompositeOption>(std::stoi(argv[4]));

    // Read the mesh
    fs::path meshName = vpkg.getMeshPath();

    // declare pointer to new Mesh object
    auto input = volcart::ITKMesh::New();

    // try to convert the ply to an ITK mesh
    volcart::io::PLYReader reader(meshName);
    try {
        reader.read();
        input = reader.getMesh();
    } catch (std::exception e) {
        std::cerr << e.what() << std::endl;
        exit(-1);
    };

    // Calculate sampling density
    double voxelsize = vpkg.getVoxelSize();
    double sa = volcart::meshmath::SurfaceArea(input) *
                (voxelsize * voxelsize) *
                (0.001 * 0.001);  // convert vx^2 -> mm^2;
    double densityFactor = 50;
    auto numberOfVertices =
        static_cast<uint16_t>(std::round(densityFactor * sa));

    // Convert to quad edge mesh and smooth the thing
    volcart::QuadMesh::Pointer qeRaw = volcart::QuadMesh::New();
    volcart::meshing::ITK2ITKQE(input, qeRaw);

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
    volcart::meshing::ITKQE2ITK(smoother->GetOutput(), smoothed);

    // Convert to polydata
    vtkPolyData* vtkMesh = vtkPolyData::New();
    volcart::meshing::ITK2VTK(input, vtkMesh);

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
    volcart::meshing::VTK2ITK(Cleaner->GetOutput(), itkACVD);

    // ABF flattening
    std::cout << "Computing parameterization..." << std::endl;
    volcart::texturing::AngleBasedFlattening abf(itkACVD);
    // abf.setABFMaxIterations(5);
    abf.compute();

    // Get uv map
    volcart::UVMap uvMap = abf.getUVMap();
    auto width = static_cast<int>(std::ceil(uvMap.ratio().width));
    auto height = static_cast<int>(
        std::ceil(static_cast<double>(width) / uvMap.ratio().aspect));

    std::cout << width << "x" << height << std::endl;

    volcart::texturing::CompositeTextureV2 compText(
        itkACVD, vpkg, uvMap, radius, width, height, type);

    // Setup rendering
    volcart::Rendering rendering;
    rendering.setTexture(compText.getTexture());
    rendering.setMesh(itkACVD);

    volcart::io::OBJWriter meshWriter;
    meshWriter.setPath(
        "textured-" + std::to_string(static_cast<int>(type)) + ".obj");
    meshWriter.setRendering(rendering);
    meshWriter.write();

    return EXIT_SUCCESS;
}
