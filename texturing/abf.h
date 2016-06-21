//
// Created by Seth Parker on 6/9/16.
// Angle-based Flattening implementation ported from the same in Blender
// Note: This is borrowed very heavily from Blender's implementation.

#ifndef VC_ABF_H
#define VC_ABF_H

#include <iostream>
#include <exception>
#include <memory>

#include <itkQuadEdgeMeshBoundaryEdgesMeshFunction.h>
#include "eigen_capi.h"

#include <opencv2/opencv.hpp>

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "deepCopy.h"

// This is terrible but it'll work for now - SP
#define SHIFT3(type, a, b, c) { type tmp; tmp = a; a = c; c = b; b = tmp; }

namespace volcart {
    namespace texturing {

      class abf {

      struct AngleInfo {
          QuadPointIdentifier p_id;
          QuadCellIdentifier  c_id;

          double alpha;  // Current angle
          double beta;   // Ideal/Original angle
          double weight; // Typically 1/b^2

          double bAlpha;

          double sine;
          double cosine;
      };

      typedef itk::QuadEdgeMeshBoundaryEdgesMeshFunction< volcart::QuadMesh > BoundaryExtractor;
      typedef std::shared_ptr<AngleInfo> AngleInfoPtr;
      typedef std::vector< std::shared_ptr<AngleInfo> > AngleGroup;

      struct VertexInfo {
          QuadPointIdentifier p_id;

          bool interior;

          double lambdaPlanar;
          double lambdaLength;

          cv::Vec2d uv;

          AngleGroup angles;
      };

      struct TriangleInfo {
          QuadCellIdentifier c_id;

          double bTriangle;
          double lambdaTriangle;
          double bstar;
          double dstar;

          AngleGroup angles;
      };

      public:
          ///// Constructors/Destructors /////
          abf();
          abf( VC_MeshType::Pointer mesh );
          ~abf();

          ///// Access Functions /////
          // Set inputs
          void setMesh( VC_MeshType::Pointer mesh );

          // Get outputs
          VC_MeshType::Pointer getMesh();
          volcart::UVMap getUVMap();

          ///// Parameters /////
          void setUseABF( bool a );
          void setABFMaxIterations( int i );

          ///// Process /////
          void compute();
      private:

          ///// Setup /////
          void _fillHalfEdgeMesh();

          // Returns the angle between AB & AC
          double _vec_angle(volcart::QuadPoint A, volcart::QuadPoint B, volcart::QuadPoint C);

          ///// Solve - ABF /////
          void _solve_abf();
          void _scale();

          void   _computeSines();
          double _computeGradient();
          double _computeGradientAlpha(HalfEdgeMesh::FacePtr face, HalfEdgeMesh::EdgePtr e0);
          double _computeSinProduct( HalfEdgeMesh::VertPtr v );
          double _computeSinProduct( HalfEdgeMesh::VertPtr v, HalfEdgeMesh::IDType a_id );
          bool   _invertMatrix();

          // Parameters
          bool    _useABF;           // If false, only compute LSCM parameterization [default: true]
          int     _maxABFIterations; // Max number of iterations
          double  _limit;            // Minimization limit

          ///// Helper functions - ABF /////
          double _sumIncidentAlphas( HalfEdgeMesh::VertPtr v );
          double _sumIncidentBetas ( HalfEdgeMesh::VertPtr v );
          double _sumTriangleAlphas( volcart::QuadCellIdentifier  c );
          double _sumTriangleBetas ( volcart::QuadCellIdentifier  c );

          ///// LSCM Loop /////
          void _solve_lscm();

          ///// Helper Functions - LSCM /////
          std::pair<HalfEdgeMesh::IDType, HalfEdgeMesh::IDType> _getMinMaxPointIDs();
          void _computePinUV();

          ///// Storage /////
          VC_MeshType::Pointer  _mesh;
          HalfEdgeMesh _heMesh;

          // Interior Vertices
          // < id in quadMesh, id in list >
          std::map< volcart::QuadPointIdentifier, volcart::QuadPointIdentifier > _interior;

          std::vector<double> _bInterior;
          cv::Mat _J2dt;

          // Pinned Point IDs
          HalfEdgeMesh::IDType _pin0;
          HalfEdgeMesh::IDType _pin1;

      };

    }// texturing
}//volcart

#endif //VC_ABF_H
