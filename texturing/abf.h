//
// Created by Seth Parker on 6/9/16.
// Angle-based Flattening implementation ported from the same in Blender
// Note: This is borrowed very heavily from Blender's implementation.

#ifndef VC_ABF_H
#define VC_ABF_H

#include <iostream>
#include <exception>
#include <memory>

#include "itkQuadEdgeMeshBoundaryEdgesMeshFunction.h"

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "deepCopy.h"

namespace volcart {
    namespace texturing {

      class abf {

      struct AngleInfo {
          QuadPointIdentifier p_id;
          QuadCellIdentifier  c_id;

          double alpha;  // Current angle
          double beta;   // Ideal/Original angle
          double weight; // Typically 1/b^2

          double sine;
          double cosine;
      };

      typedef itk::QuadEdgeMeshBoundaryEdgesMeshFunction< volcart::QuadMesh > BoundaryExtractor;
      typedef std::vector< std::shared_ptr<AngleInfo> > AngleGroup;

      public:
          ///// Constructors/Destructors /////
          abf(int maxIterations) : _maxIterations(maxIterations){};
          abf( VC_MeshType::Pointer input );
          ~abf(){};

          ///// Access Functions /////
          // Set inputs
          void setMesh( VC_MeshType::Pointer input );

          // Get outputs
          VC_MeshType::Pointer getMesh();
          volcart::UVMap getUVMap();

          ///// Process /////
          void compute();
      private:

          // Setup //
          void _fillQuadEdgeMesh();
          void _scale();

          // Solve //
          void _solve();

          void   _computeSines();
          double _computeGradient();
          bool   _invertMatrix();

          // Helper functions
          double _sumIncidentBetas ( volcart::QuadPointIdentifier p );

          const VC_MeshType::Pointer  _mesh;
          volcart::QuadMesh::Pointer  _quadMesh;

          // Boundary and Interior Vertices
          // < id in quadMesh, id in list >
          std::map< volcart::QuadPointIdentifier, volcart::QuadPointIdentifier > _boundary;
          std::map< volcart::QuadPointIdentifier, volcart::QuadPointIdentifier > _interior;

          // Reference to angles for every vertex and face
          std::map< volcart::QuadPointIdentifier, AngleGroup > _vertexAngles;
          std::map< volcart::QuadCellIdentifier , AngleGroup > _faceAngles;

          // Parameters
          int     _maxIterations; // Max number of iterations
          double  _limit;         // Minimization limit
      };

    }// texturing
}//volcart

#endif //VC_ABF_H
