//
// Created by Hannah Hatch on 7/25/16.
//

#include <iostream>
#include <vector>
#include <itkMesh.h>
#include <vc_defines.h>
#include <itkMeshFileWriter.h>
#include <itkFileTools.h>
#include "vc_datatypes.h"
#include "orderedResampling.h"
#include "io/ply2itk.h"
#include "io/objWriter.h"
#include "shapes.h"


typedef itk::MeshFileWriter<VC_MeshType> WriterType;


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Missing Argument. Use: Filename" << std::endl;
        exit(EXIT_FAILURE);

    } //Check args

    volcart::shapes::Plane plane;
    //Imports mesh from file
    VC_MeshType::Pointer plane_mesh = plane.itkMesh();

    int width = plane.orderedWidth();
    int height = plane.orderedHeight();

//    bool open_test = volcart::io::ply2itkmesh(argv[1], mesh, width, height);
//
//    if (!open_test) {
//        std::cerr << "Error Opening Input file" << std::endl;
//        exit(EXIT_FAILURE);
//    }//check file validity

    volcart::meshing::orderedResampling NoParams;
    NoParams.setMesh(plane_mesh, width, height);
    NoParams.compute();
    VC_MeshType::Pointer output = NoParams.getOutputMesh();


    volcart::io::objWriter writer;
    writer.setPath("PlaneOrderedResampling.obj");
    writer.setMesh(output);
    try
    {
        writer.writeOBJ();
    }
    catch(itk::ExceptionObject & error)
    {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }

    volcart::shapes::Arch arch;
    VC_MeshType::Pointer arch_mesh = arch.itkMesh();
    width = arch.orderedWidth();
    height = arch.orderedHeight();

    volcart::meshing::orderedResampling Params(arch_mesh, width, height);
    Params.compute();
    output = NoParams.getOutputMesh();

    writer.setPath("ArchOrderedResampling.obj");
    writer.setMesh(output);
    try
    {
        writer.writeOBJ();
    }
    catch(itk::ExceptionObject & error)
    {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
} //main