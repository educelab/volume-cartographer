//
// Created by Ryan Taber on 11/30/15.
//

/*
 * Purpose: Run volcart::meshing::rayTrace() for all volcart::shapes and write
 * results to file.
 *          Saved files will be read in by the rayTraceTest.cpp file under
 *          v-c/testing/meshing.
 */

#include "common/shapes/Arch.h"
#include "common/shapes/Cone.h"
#include "common/shapes/Cube.h"
#include "common/shapes/Plane.h"
#include "common/shapes/Sphere.h"
#include "common/vc_defines.h"
#include "meshing/rayTrace.h"

int main()
{

    // Init Shape Meshes
    volcart::ITKMesh::Pointer in_PlaneMesh, in_ArchMesh, in_CubeMesh,
        in_SphereMesh, in_ConeMesh;
    volcart::shapes::Plane Plane;
    volcart::shapes::Arch Arch;
    volcart::shapes::Cube Cube;
    volcart::shapes::Sphere Sphere;
    volcart::shapes::Cone Cone;

    // Fill the various meshes
    in_PlaneMesh = Plane.itkMesh();
    in_ArchMesh = Arch.itkMesh();
    in_CubeMesh = Cube.itkMesh();
    in_SphereMesh = Sphere.itkMesh();
    in_ConeMesh = Cone.itkMesh();

    // Int all the variables needed for the call to rayTrace()
    std::vector<cv::Vec6f> TraceResults;
    std::map<int, cv::Vec2d> UVMap;
    int TraceDir = 0;
    int Width = 0, Height = 0;

    std::ofstream MeshOutputFileStream;
    int NumberOfPoints;

    // Prepare to start writing loop
    int ShapeCounter = 0;

    while (ShapeCounter < 5) {

        // prepare to write plane data
        if (ShapeCounter == 0) {

            // call rayTrace()
            TraceResults = volcart::meshing::rayTrace(
                in_PlaneMesh, TraceDir, Width, Height, UVMap);

            // get total number of points in results vector
            NumberOfPoints = TraceResults.size();

            // Open file for writing
            MeshOutputFileStream.open("SavedPlaneRayTraceData.ply");
        }
        // prepare to write cube data
        else if (ShapeCounter == 1) {

            TraceResults = volcart::meshing::rayTrace(
                in_CubeMesh, TraceDir, Width, Height, UVMap);
            NumberOfPoints = TraceResults.size();
            MeshOutputFileStream.open("SavedCubeRayTraceData.ply");
        }
        // prepare to write arch data
        else if (ShapeCounter == 2) {

            TraceResults = volcart::meshing::rayTrace(
                in_ArchMesh, TraceDir, Width, Height, UVMap);
            NumberOfPoints = TraceResults.size();
            MeshOutputFileStream.open("SavedArchRayTraceData.ply");
        }
        // prepare to write sphere
        else if (ShapeCounter == 3) {

            TraceResults = volcart::meshing::rayTrace(
                in_SphereMesh, TraceDir, Width, Height, UVMap);
            NumberOfPoints = TraceResults.size();
            MeshOutputFileStream.open("SavedSphereRayTraceData.ply");
        }
        // prepare to write cone data
        else if (ShapeCounter == 4) {

            TraceResults = volcart::meshing::rayTrace(
                in_ConeMesh, TraceDir, Width, Height, UVMap);
            NumberOfPoints = TraceResults.size();
            MeshOutputFileStream.open("SavedConeRayTraceData.ply");
        }

        /*
         * ply writer scaled down from orderePCDMesher.cpp
         *
         * Not including a width, height or face data.
         * Should still be able to parse using parsingHelpers::parsePlyFile()
         *
         */

        // write header
        MeshOutputFileStream
            << "ply" << std::endl
            << "format ascii 1.0" << std::endl
            << "comment Created by particle simulation "
               "https://github.com/viscenter/registration-toolkit"
            << std::endl
            << "element vertex " << NumberOfPoints << std::endl
            << "property float x" << std::endl
            << "property float y" << std::endl
            << "property float z" << std::endl
            << "property float nx" << std::endl
            << "property float ny" << std::endl
            << "property float nz" << std::endl
            << "element face 0" << std::endl
            << "property list uchar int vertex_indices" << std::endl
            << "end_header" << std::endl;

        // write vertex information
        for (int point = 0; point < NumberOfPoints; point++) {

            // x y z nx ny nz
            MeshOutputFileStream
                << TraceResults[point](0) << " " << TraceResults[point](1)
                << " " << TraceResults[point](2) << " "
                << TraceResults[point](3) << " " << TraceResults[point](4)
                << " ";

            // Hack to get rid of "-0" values that appeared in the
            // saved file the first time this was run

            if (TraceResults[point](5) == -0) {
                MeshOutputFileStream << "0 " << std::endl;
            } else {
                MeshOutputFileStream << TraceResults[point](5) << " "
                                     << std::endl;
            }
        }

        MeshOutputFileStream.close();

        // move to the next shape
        ++ShapeCounter;

    }  // while

    return 0;
}
