// Created 2016-05-17 by Seth Parker
// Least Squares Conformal Mapping UV Generator
// Uses libigl to compute a mesh parameterization via LSCM method
#pragma once

#ifdef VC_USE_LIBIGL

#include <iostream>

#include <Eigen/Geometry>
#include "common/vc_defines.h"
#include "common/types/UVMap.h"

namespace volcart {
  namespace texturing {

    class LeastSquaresConformalMapping {
    public:
      LeastSquaresConformalMapping(){};
      LeastSquaresConformalMapping( ITKMesh::Pointer input );

      // Input/Output
      void setMesh( ITKMesh::Pointer input );
      ITKMesh::Pointer getMesh();
      volcart::UVMap getUVMap();

      // Processing
      void compute();

    private:
      void _fillEigenMatrices();
      void _emptyEigenMatrices();

      double _area(const Eigen::MatrixXd& v, const Eigen::MatrixXi& f);
      double _startingArea;

      ITKMesh::Pointer _mesh;
      Eigen::MatrixXd _vertices;
      Eigen::MatrixXi _faces;
      Eigen::MatrixXd _vertices_UV;
    };

  } // texturing
} // volcart

#endif // VC_USE_LIBIGL
