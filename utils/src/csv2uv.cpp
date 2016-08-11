// surfarea.cpp
// Seth Parker, Aug 2015

#include <iostream>
#include <fstream>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkMassProperties.h>

#include "common/vc_defines.h"
#include "common/io/plyWriter.h"
#include "meshing/itk2vtk.h"
#include "meshing/deepCopy.h"
#include "meshing/scaleMesh.h"

int main(int argc, char* argv[])
{
    if ( argc < 2 ) {
        std::cout << "Usage: vc_csv2uv [file.csv]" << std::endl;
        exit( -1 );
    }

    // Get Mesh
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName ( "decim.ply" );
    reader->Update();
    VC_MeshType::Pointer uvMap = VC_MeshType::New();
    volcart::meshing::vtk2itk(reader->GetOutput(), uvMap);

    // Get the surface area for scaling
    vtkSmartPointer<vtkMassProperties> massProperties = vtkMassProperties::New();
    massProperties->AddInputData(reader->GetOutput());
    massProperties->Update();
    double original_sa = massProperties->GetSurfaceArea();

    std::ifstream file ( argv[ 1 ] );
    std::string line;

    VC_PixelType vertex;
    unsigned long v_id = 0;

    while ( !file.eof() )
    {
        // Iterate over the three vertices in each face
        for( int v_counter = 0; v_counter < 3; ++v_counter ) {

            ///// Get the v_id from the csv
            getline( file, line, ',' );
            if( line.empty() ) continue; // Guard against empty lines
            v_id = std::stoul(line);

            ///// Read UVW from the file /////
            // U -> X
            getline( file, line, ',' );
            vertex[0] = std::abs( 1 - std::stod(line) ); // Blender UVs are rotated 180

            // V -> Z
            // If this is the last vertex for this cell, look for new line character instead
            if ( v_counter == 2 )
                getline( file, line );
            else
                getline( file, line, ',' );
            vertex[2] = std::abs( std::stod(line) ); // Blender UVs are rotated 180

            // W -> Y
            vertex[1] = 0.0;

            // Adjust the vertex in the mesh with it's new UVW position
            uvMap->SetPoint( v_id, vertex);
        }

    }
    file.close();

    // Get the current surface area
    vtkSmartPointer<vtkPolyData> uv_VTK = vtkPolyData::New();
    volcart::meshing::itk2vtk(uvMap, uv_VTK);
    massProperties = vtkMassProperties::New();
    massProperties->AddInputData(uv_VTK);
    massProperties->Update();
    double current_sa = massProperties->GetSurfaceArea();

    // Scale up the mesh
    // Note: This only works if the parameterization didn't introduce much area distortion
    double scale = std::sqrt(original_sa / current_sa);
    VC_MeshType::Pointer output = VC_MeshType::New();
    volcart::meshing::scaleMesh(uvMap, output, scale);

    // Write the uvMap
    volcart::io::plyWriter writer("uvmap.ply", output);
    writer.write();

    return 0;
} // end main

