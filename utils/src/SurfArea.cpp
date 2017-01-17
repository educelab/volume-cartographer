// surfarea.cpp
// Seth Parker, Aug 2015

#include <fstream>
#include <iostream>

#include <vtkMassProperties.h>
#include <vtkPolyData.h>
#include <vtkSmoothPolyDataFilter.h>

#include "core/io/PLYReader.h"
#include "core/types/VolumePkg.h"
#include "core/vc_defines.h"
#include "meshing/ITK2VTK.h"

namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "Usage: vc_area volpkg seg-id" << std::endl;
        exit(-1);
    }

    VolumePkg vpkg(argv[1]);
    std::string segID = argv[2];
    if (segID == "") {
        std::cerr << "ERROR: Incorrect/missing segmentation ID!" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (vpkg.getVersion() < 2) {
        std::cerr << "ERROR: Volume package is version " << vpkg.getVersion()
                  << " but this program requires a version >= 2" << std::endl;
        exit(EXIT_FAILURE);
    }
    vpkg.setActiveSegmentation(segID);
    fs::path meshName = vpkg.getMeshPath();

    // declare pointer to new Mesh object
    auto mesh = volcart::ITKMesh::New();

    // try to convert the ply to an ITK mesh
    volcart::io::PLYReader reader(meshName);
    try {
        reader.read();
        mesh = reader.getMesh();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        exit(-1);
    }

    vtkPolyData* smoothVTK = vtkPolyData::New();
    volcart::meshing::ITK2VTK(mesh, smoothVTK);

    vtkSmoothPolyDataFilter* smooth = vtkSmoothPolyDataFilter::New();
    smooth->SetInputData(smoothVTK);
    smooth->SetBoundarySmoothing(1);
    smooth->SetNumberOfIterations(10);
    smooth->SetRelaxationFactor(0.3);
    smooth->Update();

    auto massProperties = vtkMassProperties::New();
    massProperties->AddInputData(smooth->GetOutput());

    double areaVoxels = massProperties->GetSurfaceArea();
    double voxelSize = vpkg.getVoxelSize();

    long double umArea = areaVoxels * std::pow(voxelSize, 2);
    long double mmArea = umArea * std::pow(0.001, 2);
    long double cmArea = umArea * std::pow(0.0001, 2);
    long double inArea = umArea * std::pow(3.93700787e-5, 2);

    std::cout << std::endl;
    std::cout << "Area: " << segID << std::endl;
    std::cout << "-----------------------" << std::endl;
    std::cout << "     um^2: " << umArea << std::endl;
    std::cout << "     mm^2: " << mmArea << std::endl;
    std::cout << "     cm^2: " << cmArea << std::endl;
    std::cout << "     in^2: " << inArea << std::endl;

    return 0;
}  // end main
