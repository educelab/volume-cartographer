//
// Created by Ryan Taber on 11/30/15.
//

#include "rayTrace.h"
#include "vc_defines.h"
#include "shapes.h"

/*
 * Purpose: Run volcart::meshing::rayTrace() and write results to file.
 *          Saved file will be read in by the rayTraceTest.cpp file under
 *          v-c/testing/meshing.
 */

int main(){

    //First, create all the objects needed for the call to rayTrace()
    std::vector<cv::Vec6f> results;

    /*
     * TODO: need to update everything with the curved mesh stuff
     */


    // Essential data structure to return points and normals
    std::vector<cv::Vec6f> intersections;
    VC_MeshType::Pointer iMesh;
    volcart::shapes::Plane _mesh;
    std::map<int, cv::Vec2d> uvMap;

    int traceDir = 0; //default direction is anything != 1
    int width, height;

    //Fill the mesh
    iMesh = _mesh.itkMesh();

    //call rayTrace()
    results = volcart::meshing::rayTrace(iMesh, traceDir, width, height, uvMap);

    //get total number of points in results vector
    int numPoints = results.size();

    //write results to ply file

    std::ofstream meshFile;
    meshFile.open("savedRayTraceData.ply");

    std::cout << "Writing rayTrace results to file..." << std::endl;

    /*
     * ply writer scaled down from orderePCDMesher.cpp
     *
     * Not including a width, height or face data.
     * Should still be able to parse using parsingHelpers::parsePlyFile()
     *
     */

    // write header
    meshFile << "ply" << std::endl
    << "format ascii 1.0" << std::endl
    << "comment Created by particle simulation https://github.com/viscenter/registration-toolkit" << std::endl
    << "element vertex " << numPoints << std::endl
    << "property float x" << std::endl
    << "property float y" << std::endl
    << "property float z" << std::endl
    << "property float nx" << std::endl
    << "property float ny" << std::endl
    << "property float nz" << std::endl
    << "element face 0" << std::endl
    << "property list uchar int vertex_indices" << std::endl
    << "end_header" << std::endl;

    // write dimensions
    //meshFile << width << " " << height << std::endl;

    // write vertex information
    for (int i = 0; i < numPoints; i++) {

        // x y z nx ny nz
        meshFile << results[i](0) << " "
                 << results[i](1)  << " "
                 << results[i](2)  << " "
                 << results[i](3)  << " "
                 << results[i](4)  << " ";

                 // Hack to get rid of "-0" values that appeared in the
                 // saved file the first time this was run

                 if (results[i](5) == -0) {
                     meshFile << "0 " << std::endl;
                 }else{
                     meshFile << results[i](5) << " " << std::endl;
                 }
    }

    meshFile.close();

    return 0;
}
