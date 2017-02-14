//
// Created by Media Team on 6/24/15.
//

/*
 * Purpose: Run volcart::meshing::ITK2VTK() and write results to file.
 *          Run volcart::meshing::VTK2ITK() and write results to file.
 *          Saved file wills be read in by the ITK2VTKTest.cpp file under
 *          v-c/testing/meshing.
 *
 *          Need to use out plywriter because the vtkPLYwriter does not
 *          include normals when it writes to file
 */

#include <vtkPLYWriter.h>
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/shapes/Arch.hpp"
#include "vc/core/shapes/Cone.hpp"
#include "vc/core/shapes/Cube.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/shapes/Sphere.hpp"
#include "vc/core/vc_defines.hpp"
#include "vc/meshing/ITK2VTK.hpp"

void writePLYHeaderAndPoints(std::ostream& out, vtkPolyData* mesh);

int main()
{
    // init shapes --> used by both conversion directions
    volcart::shapes::Plane Plane;
    volcart::shapes::Cube Cube;
    volcart::shapes::Arch Arch;
    volcart::shapes::Sphere Sphere;
    volcart::shapes::Cone Cone;

    //
    // VTK2ITK conversions
    //

    // init volcart::ITKMesh::Pointer objects to hold the output of VTK2ITK
    // conversions
    volcart::ITKMesh::Pointer out_ITKPlane = volcart::ITKMesh::New();
    volcart::ITKMesh::Pointer out_ITKCube = volcart::ITKMesh::New();
    volcart::ITKMesh::Pointer out_ITKArch = volcart::ITKMesh::New();
    volcart::ITKMesh::Pointer out_ITKSphere = volcart::ITKMesh::New();
    volcart::ITKMesh::Pointer out_ITKCone = volcart::ITKMesh::New();

    // VTK2ITK() calls
    volcart::meshing::VTK2ITK(Plane.vtkMesh(), out_ITKPlane);
    volcart::meshing::VTK2ITK(Cube.vtkMesh(), out_ITKCube);
    volcart::meshing::VTK2ITK(Arch.vtkMesh(), out_ITKArch);
    volcart::meshing::VTK2ITK(Sphere.vtkMesh(), out_ITKSphere);
    volcart::meshing::VTK2ITK(Cone.vtkMesh(), out_ITKCone);

    //
    // write itk meshes to file
    //

    volcart::io::OBJWriter writer;

    // cycle through the shapes and write
    int ShapeCounter = 0;
    while (ShapeCounter < 5) {

        if (ShapeCounter == 0) {

            // write itk meshes
            writer.setMesh(out_ITKPlane);
            writer.setPath("VTKPlaneMeshConvertedToITK.obj");
            writer.write();
        } else if (ShapeCounter == 1) {

            writer.setMesh(out_ITKCube);
            writer.setPath("VTKCubeMeshConvertedToITK.obj");
            writer.write();
        } else if (ShapeCounter == 2) {

            writer.setMesh(out_ITKArch);
            writer.setPath("VTKArchMeshConvertedToITK.obj");
            writer.write();
        } else if (ShapeCounter == 3) {

            writer.setMesh(out_ITKSphere);
            writer.setPath("VTKSphereMeshConvertedToITK.obj");
            writer.write();
        } else if (ShapeCounter == 4) {

            writer.setMesh(out_ITKCone);
            writer.setPath("VTKConeMeshConvertedToITK.obj");
            writer.write();
        }

        ++ShapeCounter;
    }

    /* write vtk meshes to file
     *
     * This is a terrible implementation...but was having issues with scope, so
     * writing explicitly in
     * for each shape to complete the examples for ITK2VTKTest
     *
     */

    std::ofstream MeshOutputFileStream;
    int NumberOfVTKPoints, NumberOfVTKCells;

    // Prepare to start writing loop
    int VTKShapeCounter = 0;

    while (VTKShapeCounter < 5) {

        // plane
        if (VTKShapeCounter == 0) {

            vtkPolyData* out_VTKPlaneMesh = vtkPolyData::New();
            volcart::meshing::ITK2VTK(Plane.itkMesh(), out_VTKPlaneMesh);
            NumberOfVTKPoints = out_VTKPlaneMesh->GetNumberOfPoints();
            NumberOfVTKCells = out_VTKPlaneMesh->GetNumberOfCells();
            MeshOutputFileStream.open("ITKPlaneMeshConvertedToVTK.ply");

            // write header
            MeshOutputFileStream
                << "ply" << std::endl
                << "format ascii 1.0" << std::endl
                << "comment Created by particle simulation "
                   "https://github.com/viscenter/registration-toolkit"
                << std::endl
                << "element vertex " << NumberOfVTKPoints << std::endl
                << "property float x" << std::endl
                << "property float y" << std::endl
                << "property float z" << std::endl
                << "property float nx" << std::endl
                << "property float ny" << std::endl
                << "property float nz" << std::endl
                << "element face " << NumberOfVTKCells << std::endl
                << "property list uchar int vertex_indices" << std::endl
                << "end_header" << std::endl;

            for (int point = 0; point < NumberOfVTKPoints; point++) {

                // x y z
                MeshOutputFileStream
                    << out_VTKPlaneMesh->GetPoint(point)[0] << " "
                    << out_VTKPlaneMesh->GetPoint(point)[1] << " "
                    << out_VTKPlaneMesh->GetPoint(point)[2] << " ";

                MeshOutputFileStream
                    << out_VTKPlaneMesh->GetPointData()->GetNormals()->GetTuple(
                           point)[0]
                    << " "
                    << out_VTKPlaneMesh->GetPointData()->GetNormals()->GetTuple(
                           point)[1]
                    << " "
                    << out_VTKPlaneMesh->GetPointData()->GetNormals()->GetTuple(
                           point)[2]
                    << std::endl;
            }

            for (vtkIdType c_id = 0; c_id < NumberOfVTKCells; c_id++) {

                vtkCell* out_VTKCell = out_VTKPlaneMesh->GetCell(c_id);

                MeshOutputFileStream
                    << "3 " << out_VTKCell->GetPointIds()->GetId(0) << " "
                    << out_VTKCell->GetPointIds()->GetId(1) << " "
                    << out_VTKCell->GetPointIds()->GetId(2) << std::endl;
            }

        }
        // cube
        else if (VTKShapeCounter == 1) {

            vtkPolyData* out_VTKCubeMesh = vtkPolyData::New();
            volcart::meshing::ITK2VTK(Cube.itkMesh(), out_VTKCubeMesh);
            NumberOfVTKPoints = out_VTKCubeMesh->GetNumberOfPoints();
            NumberOfVTKCells = out_VTKCubeMesh->GetNumberOfCells();
            MeshOutputFileStream.open("ITKCubeMeshConvertedToVTK.ply");

            // write header
            MeshOutputFileStream
                << "ply" << std::endl
                << "format ascii 1.0" << std::endl
                << "comment Created by particle simulation "
                   "https://github.com/viscenter/registration-toolkit"
                << std::endl
                << "element vertex " << NumberOfVTKPoints << std::endl
                << "property float x" << std::endl
                << "property float y" << std::endl
                << "property float z" << std::endl
                << "property float nx" << std::endl
                << "property float ny" << std::endl
                << "property float nz" << std::endl
                << "element face " << NumberOfVTKCells << std::endl
                << "property list uchar int vertex_indices" << std::endl
                << "end_header" << std::endl;

            for (int point = 0; point < NumberOfVTKPoints; point++) {

                // x y z
                MeshOutputFileStream
                    << out_VTKCubeMesh->GetPoint(point)[0] << " "
                    << out_VTKCubeMesh->GetPoint(point)[1] << " "
                    << out_VTKCubeMesh->GetPoint(point)[2] << " ";

                MeshOutputFileStream
                    << out_VTKCubeMesh->GetPointData()->GetNormals()->GetTuple(
                           point)[0]
                    << " "
                    << out_VTKCubeMesh->GetPointData()->GetNormals()->GetTuple(
                           point)[1]
                    << " "
                    << out_VTKCubeMesh->GetPointData()->GetNormals()->GetTuple(
                           point)[2]
                    << std::endl;
            }

            for (vtkIdType c_id = 0; c_id < NumberOfVTKCells; c_id++) {

                vtkCell* out_VTKCell = out_VTKCubeMesh->GetCell(c_id);

                MeshOutputFileStream
                    << "3 " << out_VTKCell->GetPointIds()->GetId(0) << " "
                    << out_VTKCell->GetPointIds()->GetId(1) << " "
                    << out_VTKCell->GetPointIds()->GetId(2) << std::endl;
            }

        }
        // arch
        else if (VTKShapeCounter == 2) {

            vtkPolyData* out_VTKArchMesh = vtkPolyData::New();
            volcart::meshing::ITK2VTK(Arch.itkMesh(), out_VTKArchMesh);
            NumberOfVTKPoints = out_VTKArchMesh->GetNumberOfPoints();
            NumberOfVTKCells = out_VTKArchMesh->GetNumberOfCells();
            MeshOutputFileStream.open("ITKArchMeshConvertedToVTK.ply");

            // write header
            MeshOutputFileStream
                << "ply" << std::endl
                << "format ascii 1.0" << std::endl
                << "comment Created by particle simulation "
                   "https://github.com/viscenter/registration-toolkit"
                << std::endl
                << "element vertex " << NumberOfVTKPoints << std::endl
                << "property float x" << std::endl
                << "property float y" << std::endl
                << "property float z" << std::endl
                << "property float nx" << std::endl
                << "property float ny" << std::endl
                << "property float nz" << std::endl
                << "element face " << NumberOfVTKCells << std::endl
                << "property list uchar int vertex_indices" << std::endl
                << "end_header" << std::endl;

            for (int point = 0; point < NumberOfVTKPoints; point++) {

                // x y z
                MeshOutputFileStream
                    << out_VTKArchMesh->GetPoint(point)[0] << " "
                    << out_VTKArchMesh->GetPoint(point)[1] << " "
                    << out_VTKArchMesh->GetPoint(point)[2] << " ";

                MeshOutputFileStream
                    << out_VTKArchMesh->GetPointData()->GetNormals()->GetTuple(
                           point)[0]
                    << " "
                    << out_VTKArchMesh->GetPointData()->GetNormals()->GetTuple(
                           point)[1]
                    << " "
                    << out_VTKArchMesh->GetPointData()->GetNormals()->GetTuple(
                           point)[2]
                    << std::endl;
            }

            for (vtkIdType c_id = 0; c_id < NumberOfVTKCells; c_id++) {

                vtkCell* out_VTKCell = out_VTKArchMesh->GetCell(c_id);

                MeshOutputFileStream
                    << "3 " << out_VTKCell->GetPointIds()->GetId(0) << " "
                    << out_VTKCell->GetPointIds()->GetId(1) << " "
                    << out_VTKCell->GetPointIds()->GetId(2) << std::endl;
            }

        }
        // sphere
        else if (VTKShapeCounter == 3) {

            vtkPolyData* out_VTKSphereMesh = vtkPolyData::New();
            volcart::meshing::ITK2VTK(Sphere.itkMesh(), out_VTKSphereMesh);
            NumberOfVTKPoints = out_VTKSphereMesh->GetNumberOfPoints();
            NumberOfVTKCells = out_VTKSphereMesh->GetNumberOfCells();
            MeshOutputFileStream.open("ITKSphereMeshConvertedToVTK.ply");
            // write header
            MeshOutputFileStream
                << "ply" << std::endl
                << "format ascii 1.0" << std::endl
                << "comment Created by particle simulation "
                   "https://github.com/viscenter/registration-toolkit"
                << std::endl
                << "element vertex " << NumberOfVTKPoints << std::endl
                << "property float x" << std::endl
                << "property float y" << std::endl
                << "property float z" << std::endl
                << "property float nx" << std::endl
                << "property float ny" << std::endl
                << "property float nz" << std::endl
                << "element face " << NumberOfVTKCells << std::endl
                << "property list uchar int vertex_indices" << std::endl
                << "end_header" << std::endl;

            for (int point = 0; point < NumberOfVTKPoints; point++) {

                // x y z
                MeshOutputFileStream
                    << out_VTKSphereMesh->GetPoint(point)[0] << " "
                    << out_VTKSphereMesh->GetPoint(point)[1] << " "
                    << out_VTKSphereMesh->GetPoint(point)[2] << " ";

                MeshOutputFileStream << out_VTKSphereMesh->GetPointData()
                                            ->GetNormals()
                                            ->GetTuple(point)[0]
                                     << " "
                                     << out_VTKSphereMesh->GetPointData()
                                            ->GetNormals()
                                            ->GetTuple(point)[1]
                                     << " "
                                     << out_VTKSphereMesh->GetPointData()
                                            ->GetNormals()
                                            ->GetTuple(point)[2]
                                     << std::endl;
            }

            for (vtkIdType c_id = 0; c_id < NumberOfVTKCells; c_id++) {

                vtkCell* out_VTKCell = out_VTKSphereMesh->GetCell(c_id);

                MeshOutputFileStream
                    << "3 " << out_VTKCell->GetPointIds()->GetId(0) << " "
                    << out_VTKCell->GetPointIds()->GetId(1) << " "
                    << out_VTKCell->GetPointIds()->GetId(2) << std::endl;
            }

        }
        // cone
        else if (VTKShapeCounter == 4) {

            vtkPolyData* out_VTKConeMesh = vtkPolyData::New();
            volcart::meshing::ITK2VTK(Cone.itkMesh(), out_VTKConeMesh);
            NumberOfVTKPoints = out_VTKConeMesh->GetNumberOfPoints();
            NumberOfVTKCells = out_VTKConeMesh->GetNumberOfCells();
            MeshOutputFileStream.open("ITKConeMeshConvertedToVTK.ply");

            // write header
            MeshOutputFileStream
                << "ply" << std::endl
                << "format ascii 1.0" << std::endl
                << "comment Created by particle simulation "
                   "https://github.com/viscenter/registration-toolkit"
                << std::endl
                << "element vertex " << NumberOfVTKPoints << std::endl
                << "property float x" << std::endl
                << "property float y" << std::endl
                << "property float z" << std::endl
                << "property float nx" << std::endl
                << "property float ny" << std::endl
                << "property float nz" << std::endl
                << "element face " << NumberOfVTKCells << std::endl
                << "property list uchar int vertex_indices" << std::endl
                << "end_header" << std::endl;

            for (int point = 0; point < NumberOfVTKPoints; point++) {

                // x y z
                MeshOutputFileStream
                    << out_VTKConeMesh->GetPoint(point)[0] << " "
                    << out_VTKConeMesh->GetPoint(point)[1] << " "
                    << out_VTKConeMesh->GetPoint(point)[2] << " ";

                MeshOutputFileStream
                    << out_VTKConeMesh->GetPointData()->GetNormals()->GetTuple(
                           point)[0]
                    << " "
                    << out_VTKConeMesh->GetPointData()->GetNormals()->GetTuple(
                           point)[1]
                    << " "
                    << out_VTKConeMesh->GetPointData()->GetNormals()->GetTuple(
                           point)[2]
                    << std::endl;
            }

            for (vtkIdType c_id = 0; c_id < NumberOfVTKCells; c_id++) {

                vtkCell* out_VTKCell = out_VTKConeMesh->GetCell(c_id);

                MeshOutputFileStream
                    << "3 " << out_VTKCell->GetPointIds()->GetId(0) << " "
                    << out_VTKCell->GetPointIds()->GetId(1) << " "
                    << out_VTKCell->GetPointIds()->GetId(2) << std::endl;
            }
        }

        MeshOutputFileStream.close();

        // move to the next shape
        ++VTKShapeCounter;

    }  // while

    return EXIT_SUCCESS;
}
