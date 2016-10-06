//
// Created by Hannah Hatch on 7/26/16.
//

#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

#include "common/vc_defines.h"

namespace volcart
{
namespace meshing
{
class CalculateNormals
{
public:
    // Construction //
    CalculateNormals();
    CalculateNormals(ITKMesh::Pointer mesh);

    // Input/Output //
    void setMesh(ITKMesh::Pointer mesh);
    ITKMesh::Pointer getMesh() const;

    // Processing //
    void compute();

private:
    void _computeNormals();
    void _assignToMesh();

    ITKMesh::Pointer _input;
    ITKMesh::Pointer _output;

    std::vector<cv::Vec3d> _vertex_normals;  // convenience vector to store
                                             // calculated normals by p_id
};
}  // meshing
}  // volcart
