// ACVD Example Program
// Created by Seth Parker on 9/24/15.
//

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYReader.hpp"
#include "vc/meshing/ACVD.hpp"
#include "vc/meshing/ITK2VTK.hpp"

int main(int /*argc*/, char* argv[])
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
    volcart::meshing::VTK2ITK(acvdMesh, outputMesh);

    volcart::io::OBJWriter mesh_writer("acvd.obj", outputMesh);
    mesh_writer.write();

    return EXIT_SUCCESS;
}
