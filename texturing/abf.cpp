//
// Created by Seth Parker on 6/9/16.
//

#include "abf.h"


namespace volcart {
    namespace texturing {

      ///// Process //////
      void abf::compute()
      {
        _fillQuadEdgeMesh();
        // 1.5) Setup system defaults

        /* 2) Compute original angles
            - Sum angles for every vertex edge calc "scale"
            - Scale beta by "scale" and set this as alpha & beta for that edge
        */

        /* 3) If there are interior vertices:
            loop until max iterations reached - fallback to lscm if reached
              compute gradient for the mesh
              break if the grad is below the error limit
              invert the matrix - fallback to lscm if this fails
              compute sines
        */
      }

      ///// Setup /////
      void abf::_fillQuadEdgeMesh() {
        _quadMesh = volcart::QuadMesh::New();

        ///// Vertices /////
        for ( VC_PointsInMeshIterator point = _mesh->GetPoints()->Begin(); point != _mesh->GetPoints()->End(); ++point ) {
          volcart::QuadPoint p;
          p[0] = point->Value()[0];
          p[1] = point->Value()[1];
          p[2] = point->Value()[2];

          _quadMesh->AddPoint(p);
        }

        ///// Faces /////
        unsigned long v_ids[3];
        for ( VC_CellIterator cell = _mesh->GetCells()->Begin(); cell != _mesh->GetCells()->End(); ++cell ) {

          // Collect the point id's
          int i = 0;
          for ( VC_PointsInCellIterator point = cell.Value()->PointIdsBegin(); point != cell.Value()->PointIdsEnd(); ++point, ++i ) {
            v_ids[i] = *point;
          }

          _quadMesh->AddFaceTriangle( v_ids[0], v_ids[1], v_ids[2] );
        }

        ///// Compute the boundary /////
        BoundaryExtractor::Pointer extractor = BoundaryExtractor::New();
        volcart::QuadEdgeListPointer boundaryList = extractor->Evaluate( *_quadMesh );

        if ( boundaryList->empty() ) {
          std::runtime_error( "Mesh does not have border." );
        }

        volcart::QuadMeshPointIdentifier b_id(0);
        for ( auto it = boundaryList->begin(); it != boundaryList->end(); ++ it ) {
          auto eIt = (*it)->BeginGeomLnext();
          while ( eIt != (*it)->EndGeomLnext() ){
            volcart::QuadMeshQE* qe = eIt.Value();
            _boundary[ qe->GetOrigin(), b_id++ ];
          }
        }

        ///// Compute the interior points /////
        volcart::QuadMeshPointIdentifier m_id(0);
        volcart::QuadMeshPointIdentifier i_id(0);
        for ( auto it = _quadMesh->GetPoints()->Begin(); it != _quadMesh->GetPoints()->End(); ++it ) {
          m_id = it->Index();
          // If this point isn't on the boundary, it's interior
          if ( _boundary.find(m_id) == _boundary.end() )
            _interior[m_id] = i_id++;
        }
      }
        // Sorting
        // Defaults
        // Angles

      // Minimization Loop
        // compute gradient
        // invert the matrix *All the hard work*

    } // texturing
} // volcart