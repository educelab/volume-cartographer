//
// Created by Ryan Taber on 11/23/15.
//

#include "vc_defines.h"
#include "vtkPolyData.h"
#include "../testing/parsingHelpers.h"

//just a simple tester to see if the parser is working

int main(int argc, char **argv) {

    VC_MeshType::Pointer _mesh;
    volcart::shapes::Plane mesh;
    vtkPolyData *_vtk;

    _mesh = mesh.itkMesh();
    _vtk = vtkPolyData::New();
    volcart::meshing::itk2vtk(_mesh, _vtk);

    std::vector<VC_Vertex> savedVTKPoints;
    std::vector<VC_Cell> savedVTKCells;

    volcart::testing::parsePlyFile("vtk.ply", savedVTKPoints, savedVTKCells);


    for (int p = 0; p < savedVTKPoints.size(); p++) {
        cout << savedVTKPoints[p].x << " | " << savedVTKPoints[p].y << " | " << savedVTKPoints[p].z <<
        std::endl;
    }


    for (int c = 0; c < savedVTKCells.size(); c++) {
        cout << savedVTKCells[c].v1 << " | " << savedVTKCells[c].v2 << " | " << savedVTKCells[c].v3 <<
        std::endl;
    }

    return 0;
}