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
 *
 *          Code for writing adapted from here:
 *              http://docs.opencv.org/2.4/modules/core/doc/xml_yaml_persistence.html#filestorage-writeobj
 */

int main(){

    //First, create all the objects needed for the call to rayTrace()
    std::vector<cv::Vec6f> results;

    // Essential data structure to return points and normals
    std::vector<cv::Vec6f> intersections;
    VC_MeshType::Pointer iMesh;
    volcart::shapes::Plane _mesh;
    std::map<int, cv::Vec2d> uvMap;

    int traceDir = 0; //default direction is anything != 1
    int width, height;

    //Fill the mesh TODO: need to update with the curved mesh stuff
    iMesh = _mesh.itkMesh();

    //call rayTrace()
    results = volcart::meshing::rayTrace(iMesh, traceDir, width, height, uvMap);

    //get total number of points in results vector
    int numPoints = results.size();

    //write results to ply file

    std::ofstream meshFile;
    meshFile.open("savedRayTraceData.ply");

    if (!meshFile.open()){
        std::cerr << "Unable to open file for writing..." << std::endl;
        return 1;
    }

    std::cout << "Writing rayTrace results to file..." << std::endl;

    // write header
    meshFile << "ply" << std::endl
    << "format ascii 1.0" << std::endl
    << "comment Created by particle simulation https://github.com/viscenter/registration-toolkit" << std::endl
    //<< "element dimensions 1" << std::endl
    //<< "property float width" << std::endl
    //<< "property float height" << std::endl
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

        VC_Vertex v = results[i];

        meshFile << v.x << " "
                 << v.y << " "
                 << v.z << " "
                 << v.nx << " "
                 << v.ny << " "
                 << v.nz << " " << std::endl;
    }

    meshFile.close();

    return 0;
}
