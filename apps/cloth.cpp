//
// Created by Seth Parker on 3/14/16.
//

#include <iostream>

#include <vtkPLYReader.h>

#include "vc_defines.h"
#include "itk2vtk.h"
#include "clothModelingUV.h"
#include "io/objWriter.h"

int main( int argc, char* argv[] ) {

    // Get Mesh
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName ( "collision.ply" );
    reader->Update();
    VC_MeshType::Pointer mesh = VC_MeshType::New();
    volcart::meshing::vtk2itk(reader->GetOutput(), mesh);

    volcart::texturing::clothModelingUV::PinIDs unfurl, expand;
    volcart::texturing::clothModelingUV clothUV( mesh, 100, 500, 100, cv::Vec3d(0,1,0), unfurl, expand);
    clothUV.run();

//    for ( int i = 0; i < psb->m_nodes.size(); ++i) {
//        psb->m_nodes[i].m_x.setY(0);
//    }
//
//    psb->scale( btVector3(100, 100, 100) );

    VC_MeshType::Pointer output = clothUV.getMesh();
    std::string path = "inter_" + VC_DATE_TIME() + ".obj";
    volcart::io::objWriter writer(path, output);
    writer.write();


    return 0;
}