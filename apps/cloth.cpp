//
// Created by Seth Parker on 3/14/16.
//

#include <iostream>

#include <vtkPLYReader.h>

#include "vc_defines.h"
#include "itk2vtk.h"
#include "clothModelingUV.h"
#include "io/objWriter.h"
#include "compositeTextureV2.h"

void getPins( std::string path, VC_MeshType::Pointer mesh, volcart::texturing::clothModelingUV::PinIDs &pinList );

int main( int argc, char* argv[] ) {

    // Load volpkg
    VolumePkg vpkg( argv[1] );

    // Get Mesh
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName ( "0-decim.ply" );
    reader->Update();
    VC_MeshType::Pointer mesh = VC_MeshType::New();
    volcart::meshing::vtk2itk(reader->GetOutput(), mesh);

    // Get pinned points for unfurling step
    volcart::texturing::clothModelingUV::PinIDs unfurl;
    getPins( "1-unfurlPins.ply", mesh, unfurl);

    // Get pinned points for expansion step
    volcart::texturing::clothModelingUV::PinIDs expand;
    getPins( "3-expandPins.ply", mesh, expand);

    // Run the simulation
    volcart::texturing::clothModelingUV clothUV( mesh, 2500, 600, 500, unfurl, expand);

    // Run simulation
    clothUV.setGravity( 10 );
    clothUV.unfurl();
    clothUV.setGravity( -10 );
    clothUV.collide();
    clothUV.setGravity( -10 );
    clothUV.expand();

    // Write the scaled mesh
    VC_MeshType::Pointer output = clothUV.getMesh();
    std::string path = VC_DATE_TIME() + "_uvMap.obj";
    volcart::io::objWriter writer(path, output);
    writer.write();

    // Convert soft body to itk mesh
    volcart::UVMap uvMap = clothUV.getUVMap();
    int width  = std::ceil( uvMap.ratio().width );
    int height = std::ceil( uvMap.ratio().height );
    volcart::texturing::compositeTextureV2 result( mesh, vpkg, clothUV.getUVMap(), 7, width, height);
    volcart::io::objWriter objwriter("textured.obj", mesh, uvMap, result.texture().getImage(0));
    objwriter.write();

    return 0;
}

/////////// Get pinned points from file //////////
void getPins( std::string path, VC_MeshType::Pointer mesh, volcart::texturing::clothModelingUV::PinIDs &pinList ) {

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