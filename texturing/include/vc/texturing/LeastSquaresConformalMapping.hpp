// Created 2016-05-17 by Seth Parker
// Least Squares Conformal Mapping UV Generator
// Uses libigl to compute a mesh parameterization via LSCM method
#pragma once

#ifdef VC_USE_LIBIGL

#include <iostream>

#include <Eigen/Geometry>
#include "vc/core/types/UVMap.hpp"
#include "vc/core/vc_defines.hpp"

namespace volcart
{
namespace texturing
{

class LeastSquaresConformalMapping
{
public:
    LeastSquaresConformalMapping() {}
    explicit LeastSquaresConformalMapping(ITKMesh::Pointer input);

    // Input/Output
    void setMesh(const ITKMesh::Pointer& input);
    ITKMesh::Pointer getMesh();
    volcart::UVMap getUVMap();

    // Processing
    void compute();

private:
    void fill_eigen_matrices_();
    void empty_eigen_matrices_();

    double area_(const Eigen::MatrixXd& v, const Eigen::MatrixXi& f);
    double startingArea_;

    ITKMesh::Pointer mesh_;
    Eigen::MatrixXd vertices_;
    Eigen::MatrixXi faces_;
    Eigen::MatrixXd verticesUV_;
};
}  // texturing
}  // volcart

#endif  // VC_USE_LIBIGL
