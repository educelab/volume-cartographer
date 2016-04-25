//
// Created by Ryan Taber on 3/11/16.
//

#include "vc_defines.h"
#include "scaleMesh.h"
#include "shapes.h"
#include "itkMeshFileWriter.h"
#include "io/objWriter.h"

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


