// Angle-based Flattening (abf)
// Created by Seth Parker on 6/9/16.
// Angle-based Flattening implementation ported from the same in Blender
// Note: This is borrowed very heavily from Blender's implementation.

// This class attempts to find the ideal angles that minimize the angular distortion of the parameterized mesh.
// These idealized angles are then fed into a least-squares conformal maps algorithm which solves for the actual
// parameterized UV positions.
#pragma once

#include <iostream>
#include <exception>
#include <memory>
#include <itkQuadEdgeMeshBoundaryEdgesMeshFunction.h>
#include <opencv2/opencv.hpp>

#include "common/vc_defines.h"
#include "common/types/UVMap.h"
#include "common/types/HalfEdgeMesh.h"

// This is terrible but it'll work for now - SP
#define SHIFT3(type, a, b, c) { type tmp; tmp = a; a = c; c = b; b = tmp; }

namespace volcart {
    namespace texturing {

      class AngleBasedFlattening {
      public:
          ///// Constructors/Destructors /////
          AngleBasedFlattening();
          AngleBasedFlattening( MeshType::Pointer mesh );

          ///// Access Functions /////
          // Set inputs
          void setMesh( MeshType::Pointer mesh );

          // Get outputs
          MeshType::Pointer getMesh();
          volcart::UVMap getUVMap();

          ///// Parameters /////
          void setUseABF( bool a );
          void setABFMaxIterations( int i );

          ///// Process /////
          void compute();

          ///// Default values /////
          static const int DEFAULT_MAX_ABF_ITERATIONS = 8;
      private:
          ///// Setup /////
          void _fillHalfEdgeMesh();

          ///// Solve - ABF /////
          void _solve_abf();
          void _scale();

          void   _computeSines();
          double _computeGradient();
          double _computeGradientAlpha( HalfEdgeMesh::FacePtr face, HalfEdgeMesh::EdgePtr e0 );
          double _computeSinProduct( HalfEdgeMesh::VertPtr v );
          double _computeSinProduct( HalfEdgeMesh::VertPtr v, HalfEdgeMesh::IDType a_id );
          bool   _invertMatrix();

          // Parameters
          bool    _useABF;           // If false, only compute LSCM parameterization [default: true]
          int     _maxABFIterations; // Max number of iterations
          double  _limit;            // Minimization limit

          ///// LSCM Loop /////
          void _solve_lscm();

          ///// Helper Functions - LSCM /////
          std::pair<HalfEdgeMesh::IDType, HalfEdgeMesh::IDType> _getMinMaxPointIDs();
          void _computePinUV();

          ///// Storage /////
          MeshType::Pointer  _mesh;   // input mesh
          HalfEdgeMesh          _heMesh; // half-edge mesh for processing

          // Interior Vertices
          // < id in quadMesh, id in list >
          std::map<volcart::QuadPointIdentifier, volcart::QuadPointIdentifier> _interior;

          std::vector<double> _bInterior;
          cv::Mat _J2dt;

          // Pinned Point IDs
          HalfEdgeMesh::IDType _pin0;
          HalfEdgeMesh::IDType _pin1;

      };

    }// texturing
}//volcart
