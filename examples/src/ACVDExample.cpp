// ACVD Example Program
// Created by Seth Parker on 9/24/15.
//

#include "core/io/OBJWriter.h"
#include "core/io/PLYReader.h"
#include "core/vc_defines.h"
#include "meshing/ACVD.h"
#include "meshing/ITK2VTK.h"

int main(int argc, char* argv[])
{

    std::string meshName = argv[1];

    // declare pointer to new Mesh object
    auto itkMesh = volcart::ITKMesh::New();

    volcart::io::PLYReader reader(meshName);
    try {
        reader.read();
        itkMesh = reader.getMesh();
    } catch (std::exception e) {
        std::cerr << e.what() << std::endl;
        exit(EXIT_SUCCESS);
    }

    vtkPolyData* vtkMesh = vtkPolyData::New();
    volcart::meshing::ITK2VTK(itkMesh, vtkMesh);

    vtkPolyData* acvdMesh = vtkPolyData::New();
    volcart::meshing::ACVD(vtkMesh, acvdMesh, 10000);

    auto outputMesh = volcart::ITKMesh::New();
    volcart::meshing::vtk2itk(acvdMesh, outputMesh);

    volcart::io::OBJWriter mesh_writer("acvd.obj", outputMesh);
    mesh_writer.write();

    return EXIT_SUCCESS;
}
