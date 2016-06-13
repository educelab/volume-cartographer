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
        // Construct the mesh and get the angles
        _fillQuadEdgeMesh();

        // Scale beta to prevent degenerate cases
        _scale();

        // Solve the system
        _solve();
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
        double maxangle = (double)M_PI - minangle;

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
          AngleGroup face_angles;
          for (i = 0; i < 3; ++i ) {
            auto info = std::make_shared<AngleInfo>();
            info->c_id = cell.Index();
            info->p_id = v_ids[i];

            // Angles much fit within limits
            if ( angles[i] < minangle )
              angles[i] = minangle;
            else if ( angles[i] > maxangle )
              angles[i] = maxangle;

            info->alpha = info->beta = angles[i];
            info->weight = 2.0 / (angles[i] * angles[i]); // Using Blender weighting

            auto vertex_angles = _vertexAngles.find( v_ids[i] );
            if ( vertex_angles == _vertexAngles.end() ) {

              AngleGroup incident_angles;
              incident_angles.push_back(info);

              auto p = std::make_pair( v_ids[i], incident_angles );
              _vertexAngles.insert( p );

            } else {
              vertex_angles->second.push_back(info);
            }
            face_angles.push_back(info);
          }
          _faceAngles[cell.Index()] = face_angles;
        }
        // Set limit based on number of faces
        _limit = (_quadMesh->GetNumberOfCells() > 100) ? 1.0f : 0.001f;

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
      void abf::_scale() {

        for ( auto p_it = _interior.begin(); p_it != _interior.end(); ++p_it ) {
          double anglesum = 0.0, scale;

          anglesum = _sumIncidentBetas( p_it->first );
          scale = (anglesum == 0.0f) ? 0.0f : 2.0f * (double)M_PI / anglesum;

          auto point = _vertexAngles.find( p_it->first );
          for ( auto a_it = point->second.begin(); a_it != point->second.end(); ++a_it )
            (*a_it)->beta = (*a_it)->alpha = (*a_it)->beta * scale;

        }

      }

      //// Minimization Loop /////
      void abf::_solve() {
        _computeSines();

        for ( int i = 0; i < _maxIterations; ++i ) {
          double norm = _computeGradient();

          if ( norm < _limit )
            break;

          if(!_invertMatrix())
            std::runtime_error( "ABF failed to invert matrix");

          _computeSines();
        }

      }

      ///// Helpers /////
      double abf::_sumIncidentBetas(volcart::QuadPointIdentifier p) {
        double sum = 0.0;
        auto point = _vertexAngles.find( p );
        for ( auto it = point->second.begin(); it != point->second.end(); ++it )
          sum += (*it)->beta;

        return sum;
      }

      void abf::_computeSines() {
        // For every point
        for ( auto p_it = _vertexAngles.begin(); p_it != _vertexAngles.end(); ++p_it ) {
          // For every angle incident to that point
          for ( auto a_it = p_it->second.begin(); a_it != p_it->second.end(); ++a_it ) {
            (*a_it)->sine   = sin( (*a_it)->alpha );
            (*a_it)->cosine = cos( (*a_it)->alpha );
          }
        }
      }

      double abf::_computeGradient() {
        double norm = 0.0;

        //
      }

      bool abf::_invertMatrix() {

      }

    } // texturing
} // volcart