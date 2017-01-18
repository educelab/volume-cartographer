// surfarea.cpp
// Seth Parker, Aug 2015

#include <cmath>
#include <fstream>
#include <iostream>

#include <opencv2/core.hpp>
#include <vtkMassProperties.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>

#include "core/io/PLYWriter.hpp"
#include "core/vc_defines.hpp"
#include "meshing/DeepCopy.hpp"
#include "meshing/ITK2VTK.hpp"
#include "meshing/ScaleMesh.hpp"

using namespace volcart;

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: vc_csv2uv [mesh.ply] [uvmap.csv]" << std::endl;
        exit(-1);
    }

    // Get Mesh
    auto reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName(argv[1]);
    reader->Update();
    ITKMesh::Pointer uvMap = ITKMesh::New();
    volcart::meshing::VTK2ITK(reader->GetOutput(), uvMap);

    // Get the surface area for scaling
    auto massProperties = vtkMassProperties::New();
    massProperties->AddInputData(reader->GetOutput());
    massProperties->Update();
    double originalSA = massProperties->GetSurfaceArea();

    std::ifstream file(argv[2]);
    std::string line;

    ITKPixel vertex;
    uint64_t vId = 0;

    while (!file.eof()) {
        // Iterate over the three vertices in each face
        for (int vCounter = 0; vCounter < 3; ++vCounter) {

            ///// Get the v_id from the csv
            std::getline(file, line, ',');
            if (line.empty()) {
                continue;  // Guard against empty lines
            }
            vId = std::stoul(line);

            ///// Read UVW from the file /////
            // U -> X
            std::getline(file, line, ',');
            vertex[0] =
                std::abs(1 - std::stod(line));  // Blender UVs are rotated 180

            // V -> Z
            // If this is the last vertex for this cell, look for new line
            // character instead
            if (vCounter == 2) {
                std::getline(file, line);
            } else {
                std::getline(file, line, ',');
            }

            // Blender UVs are rotated 180
            vertex[2] = std::abs(std::stod(line));

            // W -> Y
            vertex[1] = 0.0;

            // Adjust the vertex in the mesh with it's new UVW position
            uvMap->SetPoint(vId, vertex);
        }
    }
    file.close();

    // Get the current surface area
    auto uvVTK = vtkPolyData::New();
    volcart::meshing::ITK2VTK(uvMap, uvVTK);
    massProperties = vtkMassProperties::New();
    massProperties->AddInputData(uvVTK);
    massProperties->Update();
    double currentSA = massProperties->GetSurfaceArea();

    // Scale up the mesh
    // Note: This only works if the parameterization didn't introduce much area
    // distortion
    double scale = std::sqrt(originalSA / currentSA);
    ITKMesh::Pointer output = ITKMesh::New();
    volcart::meshing::ScaleMesh(uvMap, output, scale);

    // Write the uvMap
    volcart::io::PLYWriter writer("uvmap.ply", output);
    writer.write();

    return 0;
}
