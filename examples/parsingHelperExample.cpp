//
// Created by Ryan Taber on 11/23/15.
//

#include "vc_defines.h"
#include "vtkPolyData.h"
#include "../testing/parsingHelpers.h"

//just a simple tester to see if the parser is working

int main() {

    /*
     * First, test the parsePlyFile() method
     */

    VC_MeshType::Pointer _mesh;
    volcart::shapes::Plane mesh;
    vtkPolyData *_vtk;

    _mesh = mesh.itkMesh();
    _vtk = vtkPolyData::New();
    volcart::meshing::itk2vtk(_mesh, _vtk);

    std::vector<VC_Vertex> savedVTKPoints;
    std::vector<VC_Cell> savedVTKCells;

    std::cerr << "Parsing PLY File..." << std::endl;
    volcart::testing::ParsingHelpers::parsePlyFile("vtk.ply", savedVTKPoints, savedVTKCells);

    std::cout << "\nPoints: \nx | y | z" << std::endl;
    for (int p = 0; p < savedVTKPoints.size(); p++) {
        cout << savedVTKPoints[p].x << " | " << savedVTKPoints[p].y << " | " << savedVTKPoints[p].z <<
        std::endl;
    }

    std::cout << "\nFaces: \nv1 | v2 | v3" << std::endl;
    for (int c = 0; c < savedVTKCells.size(); c++) {
        cout << savedVTKCells[c].v1 << " | " << savedVTKCells[c].v2 << " | " << savedVTKCells[c].v3 <<
        std::endl;
    }


    /*
     * Next, test the parseObjFile() method
     */

    VC_MeshType::Pointer itkMesh;
    volcart::shapes::Plane meshVTK;
    vtkSmartPointer<vtkPolyData> vtk;

    vtk = meshVTK.vtkMesh();
    vtkPolyData* vtkRead = vtk.GetPointer();
    itkMesh = VC_MeshType::New();
    volcart::meshing::vtk2itk(vtkRead, itkMesh);

    std::vector<VC_Vertex> savedITKPoints;
    std::vector<VC_Cell> savedITKCells;

    std::cerr << "\nParsing OBJ File..." << std::endl;
    volcart::testing::ParsingHelpers::parseObjFile("itk.obj", savedITKPoints, savedITKCells);

    std::cout << "Points: \nx | y | z | nx | ny | nz" << std::endl;

    for (int v = 0; v < savedITKPoints.size(); v++) {
        std::cout << savedITKPoints[v].x << " | " << savedITKPoints[v].y << " | " << savedITKPoints[v].z << " | "
                  << savedITKPoints[v].nx << " | " << savedITKPoints[v].ny << " | " << savedITKPoints[v].nz
                  << std::endl;
    }

    std::cout << "Faces: \nv1 | v2 | v3" << std::endl;
    for (int f = 0; f < savedITKCells.size(); f++) {
        cout << savedITKCells[f].v1 << " | " << savedITKCells[f].v2 << " | " << savedITKCells[f].v3 <<
        std::endl;
    }


    return 0;
}