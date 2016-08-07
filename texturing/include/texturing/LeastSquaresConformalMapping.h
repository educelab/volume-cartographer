// Created 2016-05-17 by Seth Parker
// Least Squares Conformal Mapping UV Generator
// Uses libigl to compute a mesh parameterization via LSCM method

#ifndef VC_LSCM_H
#define VC_LSCM_H

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
      LeastSquaresConformalMapping( VC_MeshType::Pointer input );

      // Input/Output
      void setMesh( VC_MeshType::Pointer input );
      VC_MeshType::Pointer getMesh();
      volcart::UVMap getUVMap();

      // Processing
      void compute();

    private:
      void _fillEigenMatrices();
      void _emptyEigenMatrices();

      double _area(const Eigen::MatrixXd& v, const Eigen::MatrixXi& f);
      double _startingArea;

      VC_MeshType::Pointer _mesh;
      Eigen::MatrixXd _vertices;
      Eigen::MatrixXi _faces;
      Eigen::MatrixXd _vertices_UV;
    };

  } // texturing
} // volcart

#endif // VC_USE_LIBIGL

#endif
