//
// Created by Seth Parker on 3/14/16.
//

#include <iostream>

#include <vtkPLYReader.h>

#include "vc_defines.h"
#include "itk2vtk.h"
#include "clothModelingUV.h"
#include "io/objWriter.h"

void getPins( std::string path, VC_MeshType::Pointer mesh, volcart::texturing::clothModelingUV::PinIDs pinList );

int main( int argc, char* argv[] ) {

    // Get Mesh
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName ( "decim.ply" );
    reader->Update();
    VC_MeshType::Pointer mesh = VC_MeshType::New();
    volcart::meshing::vtk2itk(reader->GetOutput(), mesh);

    // Get pinned points for unfurling step
    volcart::texturing::clothModelingUV::PinIDs unfurl;
    getPins( "unfurlPins.ply", mesh, unfurl);

    // Get pinned points for expansion step
    volcart::texturing::clothModelingUV::PinIDs expand;
    getPins( "expandPins.ply", mesh, expand);

    // Run the simulation
    volcart::texturing::clothModelingUV clothUV( mesh, 1000, 300, 100, cv::Vec3d(0,1,0), unfurl, expand);

    // Write the scaled mesh
    VC_MeshType::Pointer output = clothUV.getMesh();
    std::string path = "inter_" + VC_DATE_TIME() + "_start.obj";
    volcart::io::objWriter writer(path, output);
    writer.write();

    // Run simulation
    clothUV.run();
    path = "inter_" + VC_DATE_TIME() + "_unfurl.obj";
    output = clothUV.getMesh();
    writer.setPath( path );
    writer.setMesh( output );
    writer.write();

//    for ( int i = 0; i < psb->m_nodes.size(); ++i) {
//        psb->m_nodes[i].m_x.setY(0);
//    }
//
//    psb->scale( btVector3(100, 100, 100) );


    return 0;
}

/////////// Get pinned points from file //////////
void getPins( std::string path, VC_MeshType::Pointer mesh, volcart::texturing::clothModelingUV::PinIDs pinList ) {

    // Clear the pin list
    pinList.clear();

    // Load the pin list mesh from file
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName ( path.c_str() );
    reader->Update();
    VC_MeshType::Pointer pins = VC_MeshType::New();
    volcart::meshing::vtk2itk(reader->GetOutput(), pins);

    // Setup points locator
    typename VC_PointsLocatorType::Pointer pointsLocator = VC_PointsLocatorType::New();
    pointsLocator->SetPoints(mesh->GetPoints());
    pointsLocator->Initialize();

    // Iterate over all of the pins and find them in the mesh, add their IDs to the pinList
    for (VC_PointsInMeshIterator pin = pins->GetPoints()->Begin(); pin != pins->GetPoints()->End(); ++pin) {
        unsigned long pinID = pointsLocator->FindClosestPoint( pins->GetPoint( pin->Index() ) );
        pinList.push_back( pinID );
    }

}