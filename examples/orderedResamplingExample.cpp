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


typedef itk::MeshFileWriter<VC_MeshType> WriterType;


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Missing Argument. Use: Filename" << std::endl;
        exit(EXIT_FAILURE);

    } //Check args

    //Imports mesh from file
    VC_MeshType::Pointer mesh = VC_MeshType::New();
    int width;
    int height;

    bool open_test = volcart::io::ply2itkmesh(argv[1], mesh, width, height);

    if (!open_test) {
        std::cerr << "Error Opening Input file" << std::endl;
        exit(EXIT_FAILURE);
    }//check file validity

    volcart::meshing::orderedResampling NoParams;
    NoParams.setParamters(mesh, width, height);
    NoParams.compute();
    VC_MeshType::Pointer output = NoParams.getOutput();


    WriterType::Pointer writer = WriterType::New();
    writer -> SetFileName("NoParams.obj");
    writer -> SetInput(output);
    try
    {
        writer ->Update();
    }
    catch(itk::ExceptionObject & error)
    {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }

    volcart::meshing::orderedResampling Params(mesh, width, height);
    Params.compute();
    output=NoParams.getOutput();

    writer -> SetFileName("Params.obj");
    writer -> SetInput(output);
    try
    {
        writer ->Update();
    }
    catch(itk::ExceptionObject & error)
    {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
} //main