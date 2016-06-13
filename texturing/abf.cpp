//
// Created by Seth Parker on 6/9/16.
//

#include "abf.h"
#include "abf_math.h"

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
        // Min and Max angles
        double minangle = 1.0 * M_PI / 180.0;
        double maxangle = M_PI - minangle;

        unsigned long v_ids[3];
        double angles[3];
        for ( VC_CellIterator cell = _mesh->GetCells()->Begin(); cell != _mesh->GetCells()->End(); ++cell ) {

          // Collect the point id's
          int i = 0;
          for ( VC_PointsInCellIterator point = cell.Value()->PointIdsBegin(); point != cell.Value()->PointIdsEnd(); ++point, ++i ) {
            v_ids[i] = *point;
          }
          _quadMesh->AddFaceTriangle( v_ids[0], v_ids[1], v_ids[2] );

          // Get the angles
          angles[0] = vec_angle( _mesh->GetPoint(v_ids[0]),  _mesh->GetPoint(v_ids[1]), _mesh->GetPoint(v_ids[2]) );
          angles[1] = vec_angle( _mesh->GetPoint(v_ids[1]),  _mesh->GetPoint(v_ids[0]), _mesh->GetPoint(v_ids[2]) );
          angles[2] = vec_angle( _mesh->GetPoint(v_ids[2]),  _mesh->GetPoint(v_ids[0]), _mesh->GetPoint(v_ids[1]) );

          // Add the 3 angles to storage
          // If this vertex doesn't have an incident angles list, make one
          for (i = 0; i < 3; ++i ) {
            AngleInfo info;
            info.c_id = cell.Index();

            // Angles much fit within limits
            if ( angles[i] < minangle )
              angles[i] = minangle;
            else if ( angles[i] > maxangle )
              angles[i] = maxangle;

            info.alpha = info.beta = angles[i];
            info.weight = 2.0 / (angles[i] * angles[i]); // Using Blender weighting

            auto vertex_angles = _angles.find( v_ids[i] );
            if ( vertex_angles == _angles.end() ) {

              IncidentAngles incident_angles;
              incident_angles.push_back(info);

              auto p = std::make_pair( v_ids[i], incident_angles );
              _angles.insert( p );

            } else {
              vertex_angles->second.push_back(info);
            }
          }
        }

        ///// Compute the boundary /////
        BoundaryExtractor::Pointer extractor = BoundaryExtractor::New();
        volcart::QuadEdgeListPointer boundaryList = extractor->Evaluate( *_quadMesh );

        if ( boundaryList->empty() ) {
          std::runtime_error( "Mesh does not have border." );
        }

        volcart::QuadPointIdentifier b_id(0);
        for ( auto it = boundaryList->begin(); it != boundaryList->end(); ++ it ) {
          auto eIt = (*it)->BeginGeomLnext();
          while ( eIt != (*it)->EndGeomLnext() ){
            volcart::QuadMeshQE* qe = eIt.Value();
            _boundary[ qe->GetOrigin(), b_id++ ];
          }
        }

        ///// Compute the interior points /////
        volcart::QuadPointIdentifier mesh_id(0);
        volcart::QuadPointIdentifier interior_id(0);
        for ( auto it = _quadMesh->GetPoints()->Begin(); it != _quadMesh->GetPoints()->End(); ++it ) {
          mesh_id = it->Index();
          // If this point isn't on the boundary, it's interior
          if ( _boundary.find(mesh_id) == _boundary.end() )
            _interior[mesh_id] = interior_id++;
        }

        if ( _interior.empty() ) {
          std::runtime_error( "Mesh does not have interior points." );
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