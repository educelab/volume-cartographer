// surfarea.cpp
// Seth Parker, Aug 2015

#include <fstream>
#include <iostream>

#include <vtkMassProperties.h>
#include <vtkPolyData.h>
#include <vtkSmoothPolyDataFilter.h>

#include "vc/core/io/PLYReader.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/meshing/ITK2VTK.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "Usage: vc_area volpkg seg-id" << std::endl;
        exit(-1);
    }

    // Load the VolumePkg
    volcart::VolumePkg vpkg(argv[1]);

    // Get the segmentation
    std::string segID = argv[2];
    auto seg = vpkg.segmentation(segID);

    // Mesh the point cloud
    volcart::meshing::OrderedPointSetMesher mesher;
    mesher.setPointSet(seg->getPointSet());
    auto mesh = mesher.compute();

    auto smoothVTK = vtkPolyData::New();
    volcart::meshing::ITK2VTK(mesh, smoothVTK);

    auto smooth = vtkSmoothPolyDataFilter::New();
    smooth->SetInputData(smoothVTK);
    smooth->SetBoundarySmoothing(1);
    smooth->SetNumberOfIterations(10);
    smooth->SetRelaxationFactor(0.3);
    smooth->Update();

    auto massProperties = vtkMassProperties::New();
    massProperties->AddInputData(smooth->GetOutput());

    auto areaVoxels = massProperties->GetSurfaceArea();
    auto voxelSize = vpkg.getVoxelSize();

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
