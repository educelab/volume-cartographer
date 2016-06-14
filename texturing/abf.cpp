//
// Created by Seth Parker on 6/9/16.
//

#include "abf.h"
#include "abf_math.h"

namespace volcart {
    namespace texturing {
      ///// Constructors & Destructors /////
      abf::abf(int maxIterations) : _maxIterations(maxIterations) {};
      abf::~abf()
      {
        free(_J2dt);
      };

      ///// Process //////
      void abf::compute()
      {
        // Construct the mesh and get the angles
        _fillQuadEdgeMesh();

        // Scale beta to prevent degenerate cases
        _scale();

        // Solve the system
        _solve_abf();
        _solve_lscm();
      }

      ///// Setup /////
      void abf::_fillQuadEdgeMesh() {
        // Make sure we have clean storage
        _quadMesh = volcart::QuadMesh::New();
        _boundary.clear();
        _interior.clear();
        _vertInfo.clear();
        _faceInfo.clear();
        _bInterior.clear();
        free(_J2dt);

        ///// Vertices /////
        for ( VC_PointsInMeshIterator point = _mesh->GetPoints()->Begin(); point != _mesh->GetPoints()->End(); ++point ) {
          volcart::QuadPoint p;
          p[0] = point->Value()[0];
          p[1] = point->Value()[1];
          p[2] = point->Value()[2];

          _quadMesh->AddPoint(p);

          // Add every point to the vertex list
          VertexInfo i;
          i.p_id = point.Index();
          i.interior = false;
          i.lambdaLength = i.lambdaPlanar = 0.0;

          _vertInfo[point.Index()] = i;
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
          TriangleInfo tri;
          for (i = 0; i < 3; ++i ) {
            auto info = std::make_shared<AngleInfo>();
            info->c_id = tri.c_id = cell.Index();
            info->p_id = v_ids[i];

            // Angles much fit within limits
            if ( angles[i] < minangle )
              angles[i] = minangle;
            else if ( angles[i] > maxangle )
              angles[i] = maxangle;

            info->alpha = info->beta = angles[i];
            info->weight = 2.0 / (angles[i] * angles[i]); // Using Blender weighting

            // Add this angle to the vertex list
            _vertInfo[ v_ids[i] ].angles.push_back(info);

            // Add this angle to the triangle
            tri.angles.push_back(info);
          }
          _faceInfo[cell.Index()] = tri;
        }
        // Set limit based on number of faces
        _limit = (_quadMesh->GetNumberOfCells() > 100) ? 1.0f : 0.001f;
        malloc(sizeof(double) * _quadMesh->GetNumberOfCells() * 3);

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
            _vertInfo[ mesh_id ].interior = true;
            _vertInfo[ mesh_id ].lambdaLength = 1.0;
            _interior[ mesh_id ] = interior_id++;
        }

        if ( _interior.empty() ) {
          std::runtime_error( "Mesh does not have interior points." );
        }
        _bInterior = std::vector<double>(_interior.size() * 2, 0);

      }
      void abf::_scale() {

        for ( auto p_it = _interior.begin(); p_it != _interior.end(); ++p_it ) {
          double anglesum = 0.0, scale;

          anglesum = _sumIncidentBetas( p_it->first );
          scale = (anglesum == 0.0f) ? 0.0f : 2.0f * (double)M_PI / anglesum;

          auto point = _vertInfo.find( p_it->first );
          for ( auto a_it = point->second.angles.begin(); a_it != point->second.angles.end(); ++a_it )
            (*a_it)->beta = (*a_it)->alpha = (*a_it)->beta * scale;

        }

      }

      //// Angle Minimization Loop /////
      void abf::_solve_abf() {
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

      ///// Helpers - ABF /////
      double abf::_sumIncidentAlphas(volcart::QuadPointIdentifier p) {
        double sum = 0.0;
        auto point = _vertInfo.find( p );
        for ( auto it = point->second.angles.begin(); it != point->second.angles.end(); ++it )
          sum += (*it)->alpha;

        return sum;
      }

      double abf::_sumIncidentBetas(volcart::QuadPointIdentifier p) {
        double sum = 0.0;
        auto point = _vertInfo.find( p );
        for ( auto it = point->second.angles.begin(); it != point->second.angles.end(); ++it )
          sum += (*it)->beta;

        return sum;
      }

      void   abf::_computeSines() {
        // For every point
        for ( auto p_it = _vertInfo.begin(); p_it != _vertInfo.end(); ++p_it ) {
          // For every angle incident to that point
          for ( auto a_it = p_it->second.angles.begin(); a_it != p_it->second.angles.end(); ++a_it ) {
            (*a_it)->sine   = sin( (*a_it)->alpha );
            (*a_it)->cosine = cos( (*a_it)->alpha );
          }
        }
      }

      double abf::_computeGradient() {
        double norm = 0.0;

        // Gradient alpha per face
        for ( auto it = _faceInfo.begin(); it != _faceInfo.end(); ++ it ) {
          double gTriangle, gAlpha0, gAlpha1, gAlpha2;

          gAlpha0 = _computeGradientAlpha(it->second, 0);
          gAlpha1 = _computeGradientAlpha(it->second, 1);
          gAlpha2 = _computeGradientAlpha(it->second, 2);

          it->second.angles[0]->bAlpha = -gAlpha0;
          it->second.angles[1]->bAlpha = -gAlpha1;
          it->second.angles[2]->bAlpha = -gAlpha2;

          norm += (gAlpha0 * gAlpha0) + (gAlpha1 * gAlpha1) + (gAlpha2 * gAlpha2);

          gTriangle = it->second.angles[0]->alpha + it->second.angles[1]->alpha + it->second.angles[2]->alpha - (double)M_PI;
          it->second.bTriangle = -gTriangle;
          norm += gTriangle * gTriangle;
        }

        // Planarity check for interior verts
        for ( auto it = _vertInfo.begin(); it != _vertInfo.end(); ++it ) {
          if( it->second.interior ) {
            QuadPointIdentifier p_id = it->second.p_id;
            double gplanar = -2 * M_PI, glength;
            gplanar += _sumIncidentAlphas(it->second.p_id);

            _bInterior[p_id] = -gplanar;
            norm += gplanar * gplanar;

            glength = _computeSinProduct( p_id, -1 );
            _bInterior[_interior.size() + p_id] = -glength;
            norm += glength * glength;
          }
        }

        return norm;
      }

      double abf::_computeGradientAlpha(TriangleInfo face, int angle) {
        // Get pointers to the other points
        std::shared_ptr<AngleInfo> a0, a1, a2;
        a0 = face.angles[angle];
        switch(angle) {
          case 0:
            a1 = face.angles[1];
            a2 = face.angles[2];
            break;
          case 1:
            a1 = face.angles[0];
            a2 = face.angles[2];
            break;
          case 2:
            a1 = face.angles[0];
            a2 = face.angles[1];
            break;
        }

        double deriv = ( a0->alpha - a0->beta ) * a0->weight;
        deriv += _faceInfo[a0->c_id].lambdaTriangle;

        if ( _vertInfo[a0->p_id].interior ) {
          deriv += _vertInfo[a0->p_id].lambdaPlanar;
        }

        if ( _vertInfo[a1->p_id].interior ) {
          double product = _computeSinProduct( a1->p_id, a0->p_id );
          deriv += _vertInfo[a1->p_id].lambdaLength * product;
        }

        if ( _vertInfo[a2->p_id].interior ) {
          double product = _computeSinProduct( a2->p_id, a0->p_id );
          deriv += _vertInfo[a2->p_id].lambdaLength * product;
        }

        return deriv;
      }

      double abf::_computeSinProduct( QuadPointIdentifier p_id, int aid) {
        double sin1, sin2;
        sin1 = sin2 = 1.0;

        // For each incident angle on this vertex
        AngleInfoPtr a1, a2;
        for (auto a_it = _vertInfo[p_id].angles.begin(); a_it != _vertInfo[p_id].angles.end(); ++a_it ) {
          auto c_id = (*a_it)->c_id;

          // Get the other angles for the incident triangle
          if (_faceInfo[c_id].angles[0]->p_id == p_id)
          {
            a1 = _faceInfo[c_id].angles[1];
            a2 = _faceInfo[c_id].angles[2];
          }
          else if ( _faceInfo[c_id].angles[1]->p_id == p_id )
          {
            a1 = _faceInfo[c_id].angles[0];
            a2 = _faceInfo[c_id].angles[2];
          }
          else if ( _faceInfo[c_id].angles[2]->p_id == p_id )
          {
            a1 = _faceInfo[c_id].angles[0];
            a2 = _faceInfo[c_id].angles[1];
          }

          // Compute the sin product
          if ( aid == a1->p_id ) {
            sin1 *= a1->cosine;
            sin2 = 0.0;
          } else
            sin1 *= a1->sine;

          if ( aid == a2->p_id ) {
            sin1 = 0.0;
            sin2 *= a2->cosine;
          } else
            sin2 *= a2->sine;
        }

        return (sin1 - sin2);
      }

      bool   abf::_invertMatrix() {
        // Create a new solver + context
        bool success;
        LinearSolver *context;
        QuadPointIdentifier ninterior = _interior.size();

        context = EIG_linear_solver_new(0, ninterior * 2, 1);

        // Add the _bInterior points to RHS
        for( int i = 0; i < _bInterior.size(); ++i )
          EIG_linear_solver_right_hand_side_add(context, 0, i, _bInterior[i] );

        // For each face
        for( auto f_it = _faceInfo.begin(); f_it != _faceInfo.end(); ++f_it ) {
          // Setup a matrix
          double wi1, wi2, wi3, b, si, beta[3], j2[3][3], W[3][3];
          double row1[6], row2[6], row3[6];
          QuadPointIdentifier vid[6];

          auto a0 = f_it->second.angles[0];
          auto a1 = f_it->second.angles[1];
          auto a2 = f_it->second.angles[2];

          wi1 = 1.0f / a0->weight;
          wi2 = 1.0f / a1->weight;
          wi3 = 1.0f / a2->weight;

          /* bstar1 = (J1*dInv*bAlpha - bTriangle) */
          b = a0->bAlpha * wi1;
          b += a1->bAlpha * wi2;
          b += a2->bAlpha * wi3;
          b -= f_it->second.bTriangle;

          /* si = J1*d*J1t */
          si = 1.0f / (wi1 + wi2 + wi3);

          /* J1t*si*bstar1 - bAlpha */
          beta[0] = b * si - a0->bAlpha;
          beta[1] = b * si - a1->bAlpha;
          beta[2] = b * si - a2->bAlpha;

          /* use this later for computing other lambda's */
          f_it->second.bstar = b;
          f_it->second.dstar = si;

          /* set matrix */
          W[0][0] = si - a0->weight;
          W[0][1] = si;
          W[0][2] = si;
          W[1][0] = si;
          W[1][1] = si - a1->weight;
          W[1][2] = si;
          W[2][0] = si;
          W[2][1] = si;
          W[2][2] = si - a2->weight;

          vid[0] = vid[1] = vid[2] = vid[3] = vid[4] = vid[5] = -1;

          // Add each vert to RHS if interior
          if (_vertInfo[a0->p_id].interior) {
            vid[0] = a0->p_id;
            vid[3] = ninterior + a0->p_id;

            _J2dt[a0->p_id][0] = j2[0][0] = 1.0f * wi1;
            _J2dt[a1->p_id][0] = j2[1][0] = _computeSinProduct(a0->p_id, a1->p_id) * wi2;
            _J2dt[a2->p_id][0] = j2[2][0] = _computeSinProduct(a0->p_id, a2->p_id) * wi3;

            EIG_linear_solver_right_hand_side_add(context, 0, a0->p_id, j2[0][0] * beta[0]);
            EIG_linear_solver_right_hand_side_add(context, 0, ninterior + a0->p_id,
                                                  j2[1][0] * beta[1] + j2[2][0] * beta[2]);

            row1[0] = j2[0][0] * W[0][0];
            row2[0] = j2[0][0] * W[1][0];
            row3[0] = j2[0][0] * W[2][0];

            row1[3] = j2[1][0] * W[0][1] + j2[2][0] * W[0][2];
            row2[3] = j2[1][0] * W[1][1] + j2[2][0] * W[1][2];
            row3[3] = j2[1][0] * W[2][1] + j2[2][0] * W[2][2];
          }

          if (_vertInfo[a1->p_id].interior) {
            vid[1] = a1->p_id;
            vid[4] = ninterior + a1->p_id;

            _J2dt[a0->p_id][1] = j2[0][1] = _computeSinProduct(a1->p_id, a0->p_id) * wi1;
            _J2dt[a1->p_id][1] = j2[1][1] = 1.0f * wi2;
            _J2dt[a2->p_id][1] = j2[2][1] = _computeSinProduct(a1->p_id, a2->p_id) * wi3;

            EIG_linear_solver_right_hand_side_add(context, 0, a1->p_id, j2[1][1] * beta[1]);
            EIG_linear_solver_right_hand_side_add(context, 0, ninterior + a1->p_id,
                                                  j2[0][1] * beta[0] + j2[2][1] * beta[2]);

            row1[1] = j2[1][1] * W[0][1];
            row2[1] = j2[1][1] * W[1][1];
            row3[1] = j2[1][1] * W[2][1];

            row1[4] = j2[0][1] * W[0][0] + j2[2][1] * W[0][2];
            row2[4] = j2[0][1] * W[1][0] + j2[2][1] * W[1][2];
            row3[4] = j2[0][1] * W[2][0] + j2[2][1] * W[2][2];
          }

          if (_vertInfo[a2->p_id].interior) {
            vid[2] = a2->p_id;
            vid[5] = ninterior + a2->p_id;

            _J2dt[a0->p_id][2] = j2[0][2] = _computeSinProduct(a2->p_id, a0->p_id) * wi1;
            _J2dt[a1->p_id][2] = j2[1][2] = _computeSinProduct(a2->p_id, a1->p_id) * wi2;
            _J2dt[a2->p_id][2] = j2[2][2] = 1.0f * wi3;

            EIG_linear_solver_right_hand_side_add(context, 0, a2->p_id, j2[2][2] * beta[2]);
            EIG_linear_solver_right_hand_side_add(context, 0, ninterior + a2->p_id,
                                                  j2[0][2] * beta[0] + j2[1][2] * beta[1]);

            row1[2] = j2[2][2] * W[0][2];
            row2[2] = j2[2][2] * W[1][2];
            row3[2] = j2[2][2] * W[2][2];

            row1[5] = j2[0][2] * W[0][0] + j2[1][2] * W[0][1];
            row2[5] = j2[0][2] * W[1][0] + j2[1][2] * W[1][1];
            row3[5] = j2[0][2] * W[2][0] + j2[1][2] * W[2][1];
          }

          for (int i = 0; i < 3; ++i) {
            QuadPointIdentifier r = vid[i];

            if (r == -1)
              continue;

            for (int j = 0; j < 6; ++j) {
              QuadPointIdentifier c = vid[j];

              if (c == -1)
                continue;

              if (i == 0)
                EIG_linear_solver_matrix_add(context, r, c, j2[0][i] * row1[j]);
              else
                EIG_linear_solver_matrix_add(context, r + ninterior, c, j2[0][i] * row1[j]);

              if (i == 1)
                EIG_linear_solver_matrix_add(context, r, c, j2[1][i] * row2[j]);
              else
                EIG_linear_solver_matrix_add(context, r + ninterior, c, j2[1][i] * row2[j]);


              if (i == 2)
                EIG_linear_solver_matrix_add(context, r, c, j2[2][i] * row3[j]);
              else
                EIG_linear_solver_matrix_add(context, r + ninterior, c, j2[2][i] * row3[j]);
            }
          }
        }

        // solve
        success = EIG_linear_solver_solve(context);

        // if successful, update
        if (success) {
          for (auto f_it = _faceInfo.begin(); f_it != _faceInfo.end(); ++f_it) {
            double dlambda1, pre[3], dalpha;

            auto a0 = f_it->second.angles[0];
            auto a1 = f_it->second.angles[1];
            auto a2 = f_it->second.angles[2];

            pre[0] = pre[1] = pre[2] = 0.0;

            if (_vertInfo[a0->p_id].interior) {
              double x  =  EIG_linear_solver_variable_get(context, 0, a0->p_id);
              double x2 = EIG_linear_solver_variable_get(context, 0, ninterior + a0->p_id);
              pre[0] += _J2dt[a0->p_id][0] * x;
              pre[1] += _J2dt[a1->p_id][0] * x2;
              pre[2] += _J2dt[a2->p_id][0] * x2;
            }

            if (_vertInfo[a1->p_id].interior) {
              double x  = EIG_linear_solver_variable_get(context, 0, a1->p_id);
              double x2 = EIG_linear_solver_variable_get(context, 0, ninterior + a1->p_id);
              pre[0] += _J2dt[a0->p_id][1] * x2;
              pre[1] += _J2dt[a1->p_id][1] * x;
              pre[2] += _J2dt[a2->p_id][1] * x2;
            }

            if (_vertInfo[a2->p_id].interior) {
              double x  = EIG_linear_solver_variable_get(context, 0, a2->p_id);
              double x2 = EIG_linear_solver_variable_get(context, 0, ninterior + a2->p_id);
              pre[0] += _J2dt[a0->p_id][2] * x2;
              pre[1] += _J2dt[a1->p_id][2] * x2;
              pre[2] += _J2dt[a2->p_id][2] * x;
            }

            dlambda1 = pre[0] + pre[1] + pre[2];
            dlambda1 = f_it->second.dstar * (f_it->second.bstar - dlambda1);

            f_it->second.bTriangle += dlambda1;

            dalpha = (a0->bAlpha - dlambda1);
            a0->alpha += dalpha / a0->weight - pre[0];

            dalpha = (a1->bAlpha - dlambda1);
            a1->alpha += dalpha / a1->weight - pre[1];

            dalpha = (a2->bAlpha - dlambda1);
            a2->alpha += dalpha / a2->weight - pre[2];

            /* clamp */
            for(int i = 0; i < 3; ++i ) {
              if( f_it->second.angles[i]->alpha > (double)M_PI )
                f_it->second.angles[i]->alpha = (double)M_PI;
              else if( f_it->second.angles[i]->alpha < 0.0f )
                f_it->second.angles[i]->alpha = 0.0f;
            }

          }

          QuadPointIdentifier p_id;
          for ( auto it = _interior.begin(); it != _interior.end(); ++it ) {
            p_id = _vertInfo[it->second].p_id;
            _vertInfo[it->second].lambdaPlanar += EIG_linear_solver_variable_get(context, 0, p_id);
            _vertInfo[it->second].lambdaLength += EIG_linear_solver_variable_get(context, 0, ninterior + p_id);
          }
        }

        // delete context
        EIG_linear_solver_delete(context);

        // return success state
        return success;
      }

      ///// LSCM Loop /////
      void abf::_solve_lscm()
      {
        // find two pins
          // extrema?

        // Setup solver context

        // Add pins to RHS

        // Use area to determine faces flip

        // Construct matrix
          // For each face
            // Flip faces

        // Find max sin from angles
        // Shift verts for stable order?

        // Setup angle based lscm

        // Solve
          // Success - load solution
          // Failure - manually set uv's

        // Cleanup context

      }


    } // texturing
} // volcart