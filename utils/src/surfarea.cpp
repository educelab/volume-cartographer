// surfarea.cpp
// Seth Parker, Aug 2015

#include <fstream>
#include <iostream>

#include "core/io/PLYReader.h"
#include "core/types/VolumePkg.h"
#include "core/vc_defines.h"
#include "meshing/itk2vtk.h"

#include <vtkMassProperties.h>
#include <vtkPolyData.h>
#include <vtkSmoothPolyDataFilter.h>

namespace fs = boost::filesystem;

int main(int argc, char* argv[])
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
    } catch (volcart::IOException e) {
        std::cerr << e.what() << std::endl;
        exit(-1);
    }

    vtkPolyData* smoothVTK = vtkPolyData::New();
    volcart::meshing::itk2vtk(mesh, smoothVTK);

    vtkSmoothPolyDataFilter* smooth = vtkSmoothPolyDataFilter::New();
    smooth->SetInputData(smoothVTK);
    smooth->SetBoundarySmoothing(true);
    smooth->SetNumberOfIterations(10);
    smooth->SetRelaxationFactor(0.3);
    smooth->Update();

    vtkSmartPointer<vtkMassProperties> massProperties =
        vtkMassProperties::New();

    massProperties->AddInputData(smooth->GetOutput());

    double area_voxels = massProperties->GetSurfaceArea();
    double voxel_size = vpkg.getVoxelSize();

    long double area_um = area_voxels * powl(voxel_size, 2);
    long double area_mm = area_um * powl(0.001, 2);
    long double area_cm = area_um * powl(0.0001, 2);
    long double area_in = area_um * powl(3.93700787e-5, 2);

    std::cout << std::endl;
    std::cout << "Area: " << segID << std::endl;
    std::cout << "-----------------------" << std::endl;
    std::cout << "     um^2: " << area_um << std::endl;
    std::cout << "     mm^2: " << area_mm << std::endl;
    std::cout << "     cm^2: " << area_cm << std::endl;
    std::cout << "     in^2: " << area_in << std::endl;

    return 0;
}  // end main
