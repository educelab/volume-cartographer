//
// Created by Seth Parker on 10/22/15.
//

#include "vc/texturing/SimpleUV.hpp"

using namespace volcart::texturing;

volcart::UVMap volcart::texturing::SimpleUV(
    const ITKMesh::Pointer& mesh, int width, int height)
{
    volcart::UVMap uvMap;
    uint64_t pointID, arrayX, arrayY;
    double u, v;

    // Account for zero indexing of points
    auto maxIndexX = static_cast<double>(width - 1);
    auto maxIndexY = static_cast<double>(height - 1);

    // Generate UV coord for each point in mesh
    auto point = mesh->GetPoints()->Begin();
    while (point != mesh->GetPoints()->End()) {
        pointID = point.Index();

        // Assume that the input vertices can be ordered into a 2D array of size
        // width * height
        // Calculate the point's 2D array position [ArrayX, ArrayY] based on its
        // pointID
        arrayX = pointID % width;
        arrayY = (pointID - arrayX) / width;

        // Calculate the point's UV position
        u = static_cast<double>(arrayX) / maxIndexX;
        v = static_cast<double>(arrayY) / maxIndexY;

        cv::Vec2d uv(u, v);

        // Add the uv coordinates into our map at the point index specified
        uvMap.set(pointID, uv);

        ++point;
    }

    return uvMap;
}
