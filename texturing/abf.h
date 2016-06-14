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
#include "linear_solver.h"

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
          abf(int maxIterations);
          abf( VC_MeshType::Pointer input );
          ~abf();

          ///// Access Functions /////
          // Set inputs
          void setMesh( VC_MeshType::Pointer input );

          // Get outputs
          VC_MeshType::Pointer getMesh();
          volcart::UVMap getUVMap();

          ///// Process /////
          void compute();
      private:

          ///// Setup /////
          void _fillQuadEdgeMesh();

          ///// Solve - ABF /////
          void _solve_abf();
          void _scale();

          void   _computeSines();
          double _computeGradient();
          double _computeGradientAlpha(TriangleInfo face, int angle);
          double _computeSinProduct( volcart::QuadPointIdentifier p_id, int aid = -1 );
          bool   _invertMatrix();

          // Parameters
          int     _maxIterations; // Max number of iterations
          double  _limit;         // Minimization limit

          ///// Helper functions - ABF /////
          double _sumIncidentAlphas( volcart::QuadPointIdentifier p );
          double _sumIncidentBetas ( volcart::QuadPointIdentifier p );
          double _sumTriangleAlphas( volcart::QuadCellIdentifier  c );
          double _sumTriangleBetas ( volcart::QuadCellIdentifier  c );

          ///// LSCM Loop /////
          void _solve_lscm();

          ///// Storage /////
          const VC_MeshType::Pointer  _mesh;
          volcart::QuadMesh::Pointer  _quadMesh;

          // Boundary and Interior Vertices
          // < id in quadMesh, id in list >
          std::map< volcart::QuadPointIdentifier, volcart::QuadPointIdentifier > _boundary;
          std::map< volcart::QuadPointIdentifier, volcart::QuadPointIdentifier > _interior;

          // Reference to angles for every vertex and face
          std::map< volcart::QuadPointIdentifier, VertexInfo   > _vertInfo;
          std::map< volcart::QuadCellIdentifier , TriangleInfo > _faceInfo;

          std::vector<double> _bInterior;
          double (*_J2dt)[3];
      };

    }// texturing
}//volcart

#endif //VC_ABF_H
