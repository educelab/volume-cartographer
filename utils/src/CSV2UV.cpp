// surfarea.cpp
// Seth Parker, Aug 2015

#include <fstream>
#include <iostream>
#include <math.h>

#include <opencv2/core.hpp>
#include <vtkMassProperties.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>

#include "core/io/PLYWriter.h"
#include "core/vc_defines.h"
#include "meshing/DeepCopy.h"
#include "meshing/ITK2VTK.h"
#include "meshing/ScaleMesh.h"

using namespace volcart;

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cout << "Usage: vc_csv2uv [mesh.ply] [uvmap.csv]" << std::endl;
        exit(-1);
    }

    // Get Mesh
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName(argv[1]);
    reader->Update();
    ITKMesh::Pointer uvMap = ITKMesh::New();
    volcart::meshing::VTK2ITK(reader->GetOutput(), uvMap);

    // Get the surface area for scaling
    vtkSmartPointer<vtkMassProperties> massProperties =
        vtkMassProperties::New();
    massProperties->AddInputData(reader->GetOutput());
    massProperties->Update();
    double original_sa = massProperties->GetSurfaceArea();

    std::ifstream file(argv[2]);
    std::string line;

    ITKPixel vertex;
    unsigned long v_id = 0;

    while (!file.eof()) {
        // Iterate over the three vertices in each face
        for (int v_counter = 0; v_counter < 3; ++v_counter) {

            ///// Get the v_id from the csv
            getline(file, line, ',');
            if (line.empty())
                continue;  // Guard against empty lines
            v_id = std::stoul(line);

            ///// Read UVW from the file /////
            // U -> X
            getline(file, line, ',');
            vertex[0] =
                std::abs(1 - std::stod(line));  // Blender UVs are rotated 180

            // V -> Z
            // If this is the last vertex for this cell, look for new line
            // character instead
            if (v_counter == 2)
                getline(file, line);
            else
                getline(file, line, ',');
            vertex[2] =
                std::abs(std::stod(line));  // Blender UVs are rotated 180

            // W -> Y
            vertex[1] = 0.0;

            // Adjust the vertex in the mesh with it's new UVW position
            uvMap->SetPoint(v_id, vertex);
        }
    }
    file.close();

    // Get the current surface area
    vtkSmartPointer<vtkPolyData> uv_VTK = vtkPolyData::New();
    volcart::meshing::ITK2VTK(uvMap, uv_VTK);
    massProperties = vtkMassProperties::New();
    massProperties->AddInputData(uv_VTK);
    massProperties->Update();
    double current_sa = massProperties->GetSurfaceArea();

    // Scale up the mesh
    // Note: This only works if the parameterization didn't introduce much area
    // distortion
    double scale = std::sqrt(original_sa / current_sa);
    ITKMesh::Pointer output = ITKMesh::New();
    volcart::meshing::ScaleMesh(uvMap, output, scale);

    // Write the uvMap
    volcart::io::PLYWriter writer("uvmap.ply", output);
    writer.write();

    return 0;
}  // end main
