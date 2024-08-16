// surfarea.cpp
// Seth Parker, Aug 2015

#include <fstream>
#include <iostream>

#include <vtkMassProperties.h>
#include <vtkPolyData.h>
#include <vtkSmoothPolyDataFilter.h>

#include "vc/core/io/FileFilters.hpp"
#include "vc/core/io/MeshIO.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/meshing/ITK2VTK.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"

using namespace volcart;
using namespace volcart::meshing;

auto main(const int argc, char** argv) -> int
{
    if (argc < 3) {
        std::cout << "Usage: vc_area volpkg (seg-id | mesh)" << '\n';
        exit(-1);
    }

    // Load the VolumePkg
    VolumePkg vpkg(argv[1]);

    ITKMesh::Pointer mesh;
    std::string segID;
    if (IsFileType(argv[2], {"obj", "ply"})) {
        // Load the mesh
        const auto [mesh, uv, texture] = ReadMesh(argv[2]);
    } else {
        // Get the segmentation
        segID = argv[2];
        const auto seg = vpkg.segmentation(segID);

        // Mesh the point cloud
        OrderedPointSetMesher mesher;
        mesher.setPointSet(seg->getPointSet());
        mesh = mesher.compute();
    }

    const auto smoothVTK = vtkPolyData::New();
    ITK2VTK(mesh, smoothVTK);

    const auto smooth = vtkSmoothPolyDataFilter::New();
    smooth->SetInputData(smoothVTK);
    smooth->SetBoundarySmoothing(1);
    smooth->SetNumberOfIterations(10);
    smooth->SetRelaxationFactor(0.3);
    smooth->Update();

    const auto massProperties = vtkMassProperties::New();
    massProperties->AddInputData(smooth->GetOutput());

    const auto areaVoxels = massProperties->GetSurfaceArea();
    const auto voxelSize = vpkg.volume()->voxelSize();

    const auto umArea = areaVoxels * std::pow(voxelSize, 2);
    const auto mmArea = umArea * std::pow(0.001, 2);
    const auto cmArea = umArea * std::pow(0.0001, 2);
    const auto inArea = umArea * std::pow(3.93700787e-5, 2);

    std::cout << std::endl;
    std::cout << "Area: " << segID << std::endl;
    std::cout << "-----------------------" << std::endl;
    std::cout << "     um^2: " << umArea << std::endl;
    std::cout << "     mm^2: " << mmArea << std::endl;
    std::cout << "     cm^2: " << cmArea << std::endl;
    std::cout << "     in^2: " << inArea << std::endl;

    return 0;
}  // end main
