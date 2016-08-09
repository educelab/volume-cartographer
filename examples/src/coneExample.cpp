//
// Created by Ryan Taber on 3/11/16.
//

#include "common/vc_defines.h"
#include "meshing/scaleMesh.h"
#include "common/io/objWriter.h"
#include "common/shapes/Cone.h"
#include <itkMeshFileWriter.h>

int main(){

    volcart::shapes::Cone Cone;
    VC_MeshType::Pointer in_ConeITKMesh = Cone.itkMesh();

    //write cone mesh to file
    itk::MeshFileWriter<VC_MeshType>::Pointer itkwriter = itk::MeshFileWriter<VC_MeshType>::New();

    itkwriter->SetInput(in_ConeITKMesh);
    itkwriter->SetFileTypeAsASCII();
    itkwriter->SetFileName("ConeITKWriter.obj");
    itkwriter->Write();

    volcart::io::objWriter writer;
    writer.setMesh(in_ConeITKMesh);
    writer.setPath("ConeOURWriter.obj");
    writer.write();


    return EXIT_SUCCESS;
}
