//
// Created by Seth Parker on 6/9/16.
//

#include "abf.h"

namespace volcart {
    namespace texturing {
      ///// Constructors & Destructors /////
      abf::abf() : _maxABFIterations(20), _useABF(true) {};
      abf::abf( VC_MeshType::Pointer mesh ) : _mesh(mesh), _maxABFIterations(20), _useABF(true) {};
      abf::~abf()
      {
        _boundary.clear();
        _interior.clear();
        _vertInfo.clear();
        _faceInfo.clear();
        _bInterior.clear();
      };

      ///// Access Functions /////
      void abf::setMesh( VC_MeshType::Pointer mesh ) { _mesh = mesh; };

      ///// Get Output /////
      // Get output as mesh
      VC_MeshType::Pointer abf::getMesh() {
        VC_MeshType::Pointer output = VC_MeshType::New();
        volcart::meshing::deepCopy( _mesh, output );

        // Update the point positions
        VC_PointType p;
        for ( auto it = _vertInfo.begin(); it != _vertInfo.end(); ++it ) {
          p[0] = it->second.uv[0];
          p[1] = 0;
          p[2] = it->second.uv[1];
          output->SetPoint(it->second.p_id, p);
        }

        // To-do: Recompute normals

        return output;
      }

      ///// Parameters /////
      void abf::setUseABF(bool a) { _useABF = a; };
      void abf::setABFMaxIterations(int i) { _maxABFIterations = i; };

      ///// Process //////
      void abf::compute()
      {
        // Construct the mesh and get the angles
        _fillHalfEdgeMesh();

        // Scale beta to prevent degenerate cases & solve abf
        if(_useABF) {
          _scale();
          _solve_abf();
        }

        // Solve the system
        _solve_lscm();
      }

      ///// Setup /////
      void abf::_fillHalfEdgeMesh() {
        _heMesh.clear();

        ///// Vertices /////
        for ( VC_PointsInMeshIterator point = _mesh->GetPoints()->Begin(); point != _mesh->GetPoints()->End(); ++point ) {
          _heMesh.addVert( point->Value()[0], point->Value()[1], point->Value()[2] );
        }

        ///// Faces /////
        for (VC_CellIterator cell = _mesh->GetCells()->Begin(); cell != _mesh->GetCells()->End(); ++cell) {
          // Collect the point id's
          int i = 0;
          unsigned long v_ids[3];
          for (VC_PointsInCellIterator point = cell.Value()->PointIdsBegin();
               point != cell.Value()->PointIdsEnd(); ++point, ++i) {
            v_ids[i] = *point;
          }

          _heMesh.addFace( v_ids[0], v_ids[1], v_ids[2] );
        }
        _J2dt = cv::Mat( (int)_heMesh.getNumberOfEdges(), 2, CV_64F );

        ///// Connectivity /////
        _heMesh.constructConnectedness();

        ///// Generate unique ids just for interior points /////
        _bInterior = std::vector<double>(_heMesh.getNumberOfInteriorPoints() * 2, 0);
        unsigned long interior_id = 0;
        for ( auto v = _heMesh.getInteriorBegin(); v != _heMesh.getInteriorEnd(); ++v ) {
          _interior[(*v)->id] = interior_id++;
        }
      }


      void abf::_fillQuadEdgeMesh() {
        // Make sure we have clean storage
        _quadMesh = volcart::QuadMesh::New();
        _boundary.clear();
        _interior.clear();
        _vertInfo.clear();
        _faceInfo.clear();
        _bInterior.clear();

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
          i.uv = cv::Vec2d(0,0);
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
          angles[0] = _vec_angle(_quadMesh->GetPoint(v_ids[0]), _quadMesh->GetPoint(v_ids[1]), _quadMesh->GetPoint(v_ids[2]));
          angles[1] = _vec_angle(_quadMesh->GetPoint(v_ids[1]), _quadMesh->GetPoint(v_ids[0]), _quadMesh->GetPoint(v_ids[2]));
          angles[2] = _vec_angle(_quadMesh->GetPoint(v_ids[2]), _quadMesh->GetPoint(v_ids[0]), _quadMesh->GetPoint(v_ids[1]));

          // Add the 3 angles to storage
          // If this vertex doesn't have an incident angles list, make one
          TriangleInfo tri;
          tri.lambdaTriangle = 0.0;
          for (i = 0; i < 3; ++i ) {
            auto info = std::make_shared<AngleInfo>();
            info->c_id = tri.c_id = cell.Index();
            info->p_id = v_ids[i];

            // Angles must fit within limits
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
        _J2dt = cv::Mat( (int)_quadMesh->GetNumberOfCells() * 3, 2, CV_64F );

        ///// Compute the boundary /////
        BoundaryExtractor::Pointer extractor = BoundaryExtractor::New();
        volcart::QuadEdgeListPointer boundaryList = extractor->Evaluate( *_quadMesh );

        if ( boundaryList->empty() ) {
          std::runtime_error( "Mesh does not have border." );
        }

        volcart::QuadPointIdentifier b_id(0);
        for ( auto it = boundaryList->begin(); it != boundaryList->end(); ++it ) {
          auto eIt = (*it)->BeginGeomLnext();
          while ( eIt != (*it)->EndGeomLnext() ){
            volcart::QuadMeshQE* qe = eIt.Value();
            _boundary.insert( std::pair< const QuadPointIdentifier, QuadPointIdentifier >(qe->GetOrigin(), b_id) );
            ++b_id;
            ++eIt;
          }
        }

        ///// Compute the interior points /////
        volcart::QuadPointIdentifier mesh_id(0);
        volcart::QuadPointIdentifier interior_id(0);
        for ( auto it = _quadMesh->GetPoints()->Begin(); it != _quadMesh->GetPoints()->End(); ++it ) {
          mesh_id = it->Index();
          // If this point isn't on the boundary, it's interior
          auto test = _boundary.find(mesh_id);
          if ( _boundary.find(mesh_id) == _boundary.end() ) {
            _vertInfo[mesh_id].interior = true;
            _vertInfo[mesh_id].lambdaLength = 1.0;
            _interior[mesh_id] = interior_id++;
          }
        }

        if ( _interior.empty() ) {
          std::runtime_error( "Mesh does not have interior points." );
        }
        _bInterior = std::vector<double>(_interior.size() * 2, 0);

      }

      // Returns the angle between ab and ac
      double abf::_vec_angle(volcart::QuadPoint A, volcart::QuadPoint B, volcart::QuadPoint C) {
        cv::Vec3d vec_1, vec_2;

        vec_1[0] = B[0] - A[0];
        vec_1[1] = B[1] - A[1];
        vec_1[2] = B[2] - A[2];

        vec_2[0] = C[0] - A[0];
        vec_2[1] = C[1] - A[1];
        vec_2[2] = C[2] - A[2];

        cv::normalize(vec_1, vec_1);
        cv::normalize(vec_2, vec_2);

        double dot = vec_1.dot(vec_2);

        if ( dot <= -1.0f )
          return M_PI;
        else if ( dot >= 1.0f )
          return 0.0f;
        else
          return acos(dot);
      }

      ///// ABF /////
      // Scale angles to prevent degenerate cases
      void abf::_scale() {

        for ( auto p_it = _heMesh.getInteriorBegin(); p_it != _heMesh.getInteriorEnd(); ++p_it ) {
          double anglesum = 0.0, scale;

          anglesum = _sumIncidentBetas( *p_it );
          scale = (anglesum == 0.0f) ? 0.0f : 2.0f * (double)M_PI / anglesum;

          auto e = (*p_it)->edge;
          do {
            e->angle->beta = e->angle->alpha = e->angle->beta * scale;
            e = e->next->next->pair;
          } while (e && (e != (*p_it)->edge));

        }

      }

      // Angle minimization loop
      void abf::_solve_abf() {
        _computeSines();

        double norm = 1e10;
        for ( int i = 0; i < _maxABFIterations; ++i ) {
          norm = _computeGradient();

          if ( norm < _limit )
            break;

          if( !_invertMatrix() ) {
            std::cerr << "volcart::texturing::abf: ABF failed to invert matrix after " << i << " iterations. Falling back to LSCM." << std::endl;
            std::runtime_error("ABF failed to invert matrix");
            break;
          }

          _computeSines();
        }
        std::cerr << "volcart::texturing::abf: Norm(" << norm << ") || Limit(" << _limit << ")" << std::endl;

      }

      ///// Helpers - ABF /////
      double abf::_sumIncidentAlphas(HalfEdgeMesh::VertPtr v) {
        double sum = 0.0;
        auto e = v->edge;
        do {
          sum += e->angle->beta;
          e = e->next->next->pair;
        } while ( e && ( e != v->edge) );

        return sum;
      }

      double abf::_sumIncidentBetas(HalfEdgeMesh::VertPtr v) {
        double sum = 0.0;
        auto e = v->edge;
        do {
          sum += e->angle->beta;
          e = e->next->next->pair;
        } while ( e && ( e != v->edge) );

        return sum;
      }

      void   abf::_computeSines() {
        for (auto e = _heMesh.getEdgesBegin(); e != _heMesh.getEdgesEnd(); ++e ) {
          (*e)->angle->sine   = sin((*e)->angle->alpha);
          (*e)->angle->cosine = cos((*e)->angle->alpha);
        }
      }

      double abf::_computeGradient() {
        double norm = 0.0;

        // Gradient alpha per face
        for ( auto f = _heMesh.getFacesBegin(); f != _heMesh.getFacesEnd(); ++f ) {
          double gTriangle, gAlpha0, gAlpha1, gAlpha2;
          HalfEdgeMesh::EdgePtr e0, e1, e2;
          e0 = (*f)->edge;
          e1 = e1->next;
          e2 = e2->next;
          gAlpha0 = _computeGradientAlpha((*f), e0);
          gAlpha1 = _computeGradientAlpha((*f), e1);
          gAlpha2 = _computeGradientAlpha((*f), e2);

          e0->angle->bAlpha = -gAlpha0;
          e1->angle->bAlpha = -gAlpha1;
          e2->angle->bAlpha = -gAlpha2;

          norm += (gAlpha0 * gAlpha0) + (gAlpha1 * gAlpha1) + (gAlpha2 * gAlpha2);

          gTriangle = e0->angle->alpha + e1->angle->alpha + e2->angle->alpha - M_PI;
          (*f)->bTriangle = -gTriangle;
          norm += gTriangle * gTriangle;
        }

        // Planarity check for interior verts
        for ( auto v = _heMesh.getVertsBegin(); v != _heMesh.getVertsEnd(); ++v) {
            if ( (*v)->interior() ) {
              auto i_id = _interior[(*v)->id];
              double gplanar = -2 * M_PI, glength;

              HalfEdgeMesh::EdgePtr e = (*v)->edge;
              do {
                gplanar += e->angle->alpha;
                e = e->next->next->pair;
              } while (e && (e != (*v)->edge));

              _bInterior[i_id] = -gplanar;
              norm += gplanar * gplanar;

              glength = _computeSinProduct((*v), -1);
              _bInterior[ _heMesh.getNumberOfInteriorPoints() + i_id ] = -glength;
              norm += glength + glength;
            }
        }

        return norm;
      }

      double abf::_computeGradientAlpha(HalfEdgeMesh::FacePtr f, HalfEdgeMesh::EdgePtr e0) {
        HalfEdgeMesh::VertPtr v0 = e0->vert, v1 = e0->next->vert, v2 = e0->next->next->vert;

        double deriv = ( e0->angle->alpha -  e0->angle->beta ) * e0->angle->weight;
        deriv += f->lambdaTriangle;

        if ( v0->interior() ) {
          deriv += v0->lambdaPlanar;
        }

        if ( v1->interior()) {
          double product = _computeSinProduct( v1, v0->id );
          deriv += v1->lambdaLength * product;
        }

        if ( v2->interior() ) {
          double product = _computeSinProduct( v2, v0->id );
          deriv += v2->lambdaLength * product;
        }

        return deriv;
      }

      // Edge length constraint calculation
      double abf::_computeSinProduct( HalfEdgeMesh::VertPtr v, int a_id ) {
        HalfEdgeMesh::EdgePtr e0, e1, e2;
        double sin1, sin2;
        sin1 = sin2 = 1.0;

        e0 = v->edge;
        do {
          e1 = e0->next;
          e2 = e0->next->next;

          if (a_id == e1->id) {
            /* we are computing a derivative for this angle,
             * so we use cos and drop the other part */
            sin1 *= e1->angle->cosine;
            sin2 = 0.0;
          }
          else
            sin1 *= e1->angle->sine;

          if (a_id == e2->id) {
            /* see above */
            sin1 = 0.0;
            sin2 *= e2->angle->cosine;
          }
          else
            sin2 *= e2->angle->sine;

          e0 = e0->next->next->pair;
        } while (e0 && (e0 != v->edge));

        return (sin1 - sin2);
      }

      bool   abf::_invertMatrix() {
        // Create a new solver + context
        bool success;
        LinearSolver *context;
        HalfEdgeMesh::IDType ninterior = _heMesh.getNumberOfInteriorPoints();

        context = EIG_linear_solver_new(0, ninterior * 2, 1);

        // Add the _bInterior points to RHS
        for ( auto v = _heMesh.getInteriorBegin(); v != _heMesh.getInteriorEnd(); ++v ) {
          auto i_id = _interior[(*v)->id];
          EIG_linear_solver_right_hand_side_add(context, 0, i_id, _bInterior[i_id]);
        }

        // For each face
        int counter = 0;
        for( auto f = _heMesh.getFacesBegin(); f != _heMesh.getFacesEnd(); ++f, ++counter ) {
          // Setup a matrix
          double wi1, wi2, wi3, b, si, beta[3], j2[3][3], W[3][3];
          double row1[6], row2[6], row3[6];
          int vid[6];

          HalfEdgeMesh::EdgePtr e0 = (*f)->edge, e1 = e0->next, e2 = e1->next;
          HalfEdgeMesh::VertPtr v0 = e0->vert, v1 = e1->vert, v2 = e2->vert;

          wi1 = 1.0f / e0->angle->weight;
          wi2 = 1.0f / e1->angle->weight;
          wi3 = 1.0f / e2->angle->weight;

          /* bstar1 = (J1*dInv*bAlpha - bTriangle) */
          b =  e0->angle->bAlpha * wi1;
          b += e1->angle->bAlpha * wi2;
          b += e2->angle->bAlpha * wi3;
          b -= (*f)->bTriangle;

          /* si = J1*d*J1t */
          si = 1.0f / (wi1 + wi2 + wi3);

          /* J1t*si*bstar1 - bAlpha */
          beta[0] = b * si - e0->angle->bAlpha;
          beta[1] = b * si - e1->angle->bAlpha;
          beta[2] = b * si - e2->angle->bAlpha;

          /* use this later for computing other lambda's */
          (*f)->bstar = b;
          (*f)->dstar = si;

          /* set matrix */
          W[0][0] = si - e0->angle->weight;
          W[0][1] = si;
          W[0][2] = si;
          W[1][0] = si;
          W[1][1] = si - e1->angle->weight;
          W[1][2] = si;
          W[2][0] = si;
          W[2][1] = si;
          W[2][2] = si - e2->angle->weight;

          vid[0] = vid[1] = vid[2] = vid[3] = vid[4] = vid[5] = -1;

          // Add each vert to RHS if interior
          if (v0->interior()) {
            unsigned long i_id = vid[0] = _interior[v0->id];
            vid[3] = ninterior + i_id;

            _J2dt.at< double >(e0->id, 0) = j2[0][0] = 1.0f * wi1;
            _J2dt.at< double >(e1->id, 0) = j2[1][0] = _computeSinProduct(e0->vert, e1->id) * wi2;
            _J2dt.at< double >(e2->id, 0) = j2[2][0] = _computeSinProduct(e0->vert, e2->id) * wi3;

            EIG_linear_solver_right_hand_side_add(context, 0, i_id, j2[0][0] * beta[0]);
            EIG_linear_solver_right_hand_side_add(context, 0, ninterior + i_id,
                                                  j2[1][0] * beta[1] + j2[2][0] * beta[2]);

            row1[0] = j2[0][0] * W[0][0];
            row2[0] = j2[0][0] * W[1][0];
            row3[0] = j2[0][0] * W[2][0];

            row1[3] = j2[1][0] * W[0][1] + j2[2][0] * W[0][2];
            row2[3] = j2[1][0] * W[1][1] + j2[2][0] * W[1][2];
            row3[3] = j2[1][0] * W[2][1] + j2[2][0] * W[2][2];
          }

          if (v1->interior()) {
            unsigned long i_id = vid[1] = _interior[v1->id];
            vid[4] = ninterior + i_id;

            _J2dt.at< double >(e0->id, 1) = j2[0][1] = _computeSinProduct(e1->vert, e0->id) * wi1;
            _J2dt.at< double >(e1->id, 1) = j2[1][1] = 1.0f * wi2;
            _J2dt.at< double >(e2->id, 1) = j2[2][1] = _computeSinProduct(e1->vert, e2->id) * wi3;

            EIG_linear_solver_right_hand_side_add(context, 0, i_id, j2[1][1] * beta[1]);
            EIG_linear_solver_right_hand_side_add(context, 0, ninterior + i_id,
                                                  j2[0][1] * beta[0] + j2[2][1] * beta[2]);

            row1[1] = j2[1][1] * W[0][1];
            row2[1] = j2[1][1] * W[1][1];
            row3[1] = j2[1][1] * W[2][1];

            row1[4] = j2[0][1] * W[0][0] + j2[2][1] * W[0][2];
            row2[4] = j2[0][1] * W[1][0] + j2[2][1] * W[1][2];
            row3[4] = j2[0][1] * W[2][0] + j2[2][1] * W[2][2];
          }

          if (v2->interior()) {
            unsigned long i_id = vid[2] = _interior[v2->id];
            vid[5] = ninterior + i_id;

            _J2dt.at< double >(e0->id, 2) = j2[0][2] = _computeSinProduct(e2->vert, e0->id) * wi1;
            _J2dt.at< double >(e1->id, 2) = j2[1][2] = _computeSinProduct(e2->vert, e1->id) * wi2;
            _J2dt.at< double >(e2->id, 2) = j2[2][2] = 1.0f * wi3;

            EIG_linear_solver_right_hand_side_add(context, 0, i_id, j2[2][2] * beta[2]);
            EIG_linear_solver_right_hand_side_add(context, 0, ninterior + i_id, j2[0][2] * beta[0] + j2[1][2] * beta[1]);

            row1[2] = j2[2][2] * W[0][2];
            row2[2] = j2[2][2] * W[1][2];
            row3[2] = j2[2][2] * W[2][2];

            row1[5] = j2[0][2] * W[0][0] + j2[1][2] * W[0][1];
            row2[5] = j2[0][2] * W[1][0] + j2[1][2] * W[1][1];
            row3[5] = j2[0][2] * W[2][0] + j2[1][2] * W[2][1];
          }

          for (int i = 0; i < 3; ++i) {
            int r = vid[i];

            // Unset condition
            if (r == -1)
              continue;

            for (int j = 0; j < 6; ++j) {
              int c = vid[j];

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
          for (auto f = _heMesh.getFacesBegin(); f != _heMesh.getFacesEnd(); ++f) {
            double dlambda1, pre[3], dalpha;

            HalfEdgeMesh::EdgePtr e0 = (*f)->edge, e1 = e0->next, e2 = e1->next;
            HalfEdgeMesh::VertPtr v0 = e0->vert, v1 = e1->vert, v2 = e2->vert;

            pre[0] = pre[1] = pre[2] = 0.0;

            if (v0->interior()) {
              auto i_id = _interior[v0->id];
              double x  = EIG_linear_solver_variable_get(context, 0, i_id);
              double x2 = EIG_linear_solver_variable_get(context, 0, ninterior + i_id);
              pre[0] += _J2dt.at< double >(e0->id, 0) * x;
              pre[1] += _J2dt.at< double >(e1->id, 0) * x2;
              pre[2] += _J2dt.at< double >(e2->id, 0) * x2;
            }

            if (v1->interior()) {
              auto i_id = _interior[v1->id];
              double x  = EIG_linear_solver_variable_get(context, 0, i_id);
              double x2 = EIG_linear_solver_variable_get(context, 0, ninterior + i_id);
              pre[0] += _J2dt.at< double >(e0->id, 1) * x2;
              pre[1] += _J2dt.at< double >(e1->id, 1) * x;
              pre[2] += _J2dt.at< double >(e2->id, 1) * x2;
            }

            if (v2->interior()) {
              auto i_id = _interior[v2->id];
              double x  = EIG_linear_solver_variable_get(context, 0, i_id);
              double x2 = EIG_linear_solver_variable_get(context, 0, ninterior + i_id);
              pre[0] += _J2dt.at< double >(e0->id, 2) * x2;
              pre[1] += _J2dt.at< double >(e1->id, 2) * x2;
              pre[2] += _J2dt.at< double >(e2->id, 2) * x;
            }

            dlambda1 = pre[0] + pre[1] + pre[2];
            dlambda1 = (*f)->dstar * ((*f)->bstar - dlambda1);

            (*f)->lambdaTriangle += dlambda1;

            dalpha = (e0->angle->bAlpha - dlambda1);
            e0->angle->alpha += dalpha / e0->angle->weight - pre[0];

            dalpha = (e1->angle->bAlpha - dlambda1);
            e1->angle->alpha += dalpha / e1->angle->weight - pre[1];

            dalpha = (e2->angle->bAlpha - dlambda1);
            e2->angle->alpha += dalpha / e2->angle->weight - pre[2];

            /* clamp */
            auto e = (*f)->edge;
            do {
              if (e->angle->alpha > M_PI)
                e->angle->alpha = M_PI;
              else if (e->angle->alpha < 0.0f)
                e->angle->alpha = 0.0f;
              e = e->next;
            } while (e != (*f)->edge);

          }

          QuadPointIdentifier p_id, i_id;
          for ( auto it = _interior.begin(); it != _interior.end(); ++it ) {
            p_id = it->first;
            i_id = it->second;
            _heMesh.getVert(p_id)->lambdaPlanar += EIG_linear_solver_variable_get(context, 0, i_id);
            _heMesh.getVert(p_id)->lambdaLength += EIG_linear_solver_variable_get(context, 0, ninterior + i_id);
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
        double area_pinned_up, area_pinned_down;
        bool flip_faces, result;

        // find two pins and compute their positions
        auto MinMaxPair = _getMinMaxPointIDs();
        _pin0 = MinMaxPair.first;
        _pin1 = MinMaxPair.second;
        _computePinUV();

        // Setup solver context
        LinearSolver *context;
        context = EIG_linear_least_squares_solver_new(2 * _mesh->GetNumberOfCells(), 2 * _mesh->GetNumberOfPoints(), 1);

        // Add pins to solver
        EIG_linear_solver_variable_lock(context, 2 * _vertInfo[_pin0].p_id);
        EIG_linear_solver_variable_lock(context, 2 * _vertInfo[_pin0].p_id + 1);
        EIG_linear_solver_variable_lock(context, 2 * _vertInfo[_pin1].p_id);
        EIG_linear_solver_variable_lock(context, 2 * _vertInfo[_pin1].p_id + 1);

        EIG_linear_solver_variable_set(context, 0, 2 * _vertInfo[_pin0].p_id,     _vertInfo[_pin0].uv[0]);
        EIG_linear_solver_variable_set(context, 0, 2 * _vertInfo[_pin0].p_id + 1, _vertInfo[_pin0].uv[1]);
        EIG_linear_solver_variable_set(context, 0, 2 * _vertInfo[_pin1].p_id,     _vertInfo[_pin1].uv[0]);
        EIG_linear_solver_variable_set(context, 0, 2 * _vertInfo[_pin1].p_id + 1, _vertInfo[_pin1].uv[1]);

        // Construct matrix
        int row = 0;
        for ( auto it = _faceInfo.begin(); it != _faceInfo.end(); ++it ) {
          QuadPointIdentifier v0 = it->second.angles[0]->p_id;
          QuadPointIdentifier v1 = it->second.angles[1]->p_id;
          QuadPointIdentifier v2 = it->second.angles[2]->p_id;

          double a0 = it->second.angles[0]->alpha;
          double a1 = it->second.angles[1]->alpha;
          double a2 = it->second.angles[2]->alpha;

          // Find max sin from angles
          double sin0 = sin(a0);
          double sin1 = sin(a1);
          double sin2 = sin(a2);

          double sinmax = std::max( sin0, std::max(sin1, sin2) );

          // Shift verts for stable order
          // Careful. Only use these values going forward through the loop
          if( sin2 != sinmax ) {
            SHIFT3(QuadPointIdentifier, v0, v1, v2);
            SHIFT3(double, a0, a1, a2);
            SHIFT3(double, sin0, sin1, sin2);

            if( sin1 == sinmax ) {
              SHIFT3(QuadPointIdentifier, v0, v1, v2);
              SHIFT3(double, a0, a1, a2);
              SHIFT3(double, sin0, sin1, sin2);
            }
          }

          // Setup angle based lscm
          double ratio = (sin2 == 0.0) ? 1.0 : sin1 / sin2;
          double cosine = cos(a0) * ratio;
          double sine = sin0 * ratio;

          EIG_linear_solver_matrix_add(context, row, 2 * v0,   cosine - 1.0f);
          EIG_linear_solver_matrix_add(context, row, 2 * v0 + 1, -sine);
          EIG_linear_solver_matrix_add(context, row, 2 * v1,   -cosine);
          EIG_linear_solver_matrix_add(context, row, 2 * v1 + 1, sine);
          EIG_linear_solver_matrix_add(context, row, 2 * v2,   1.0);
          row++;

          EIG_linear_solver_matrix_add(context, row, 2 * v0,   sine);
          EIG_linear_solver_matrix_add(context, row, 2 * v0 + 1, cosine - 1.0f);
          EIG_linear_solver_matrix_add(context, row, 2 * v1,   -sine);
          EIG_linear_solver_matrix_add(context, row, 2 * v1 + 1, -cosine);
          EIG_linear_solver_matrix_add(context, row, 2 * v2 + 1, 1.0);
          row++;
        }

        // Solve
        if( !EIG_linear_solver_solve(context) ) {
          std::runtime_error( "Failed to solve lscm." );
        }

        // Update UVs
        for( auto it = _vertInfo.begin(); it != _vertInfo.end(); ++it ) {
          it->second.uv[0] = EIG_linear_solver_variable_get(context, 0, 2 * it->second.p_id);
          it->second.uv[1] = EIG_linear_solver_variable_get(context, 0, 2 * it->second.p_id + 1);
        }

        // Cleanup context
        EIG_linear_solver_delete(context);
      }

      ///// Helpers - LSCM /////
      std::pair<QuadPointIdentifier, QuadPointIdentifier> abf::_getMinMaxPointIDs() {
        std::vector<double> min(3, 1e20), max(3, -1e20);
        QuadPointIdentifier minVert = 0;
        QuadPointIdentifier maxVert = 0;

        for (auto it = _mesh->GetPoints()->Begin(); it != _mesh->GetPoints()->End(); ++it )
        {

          if ( it->Value()[0] < min[0] &&
               it->Value()[1] < min[1] &&
               it->Value()[2] < min[2] )
          {
            min[0] = it->Value()[0];
            min[1] = it->Value()[1];
            min[2] = it->Value()[2];
            minVert = it->Index();
          } else if ( it->Value()[0] > max[0] &&
                      it->Value()[1] > max[1] &&
                      it->Value()[2] > max[2] )
          {
            max[0] = it->Value()[0];
            max[1] = it->Value()[1];
            max[2] = it->Value()[2];
            maxVert = it->Index();
          }
        }

        return std::make_pair(minVert, maxVert);
      }

      void abf::_computePinUV() {
        if( _pin0 == _pin1 ) {
          // Degenerate case, get two other points
          auto it = _boundary.begin();
          _pin0 = it->first;
          _pin1 = std::next(it)->first;

          _vertInfo[_pin0].uv = cv::Vec2d(0.0, 0.5);
          _vertInfo[_pin1].uv = cv::Vec2d(1.0, 0.5);
        }
        else {
          int diru, dirv, dirx, diry;

          auto pt = _mesh->GetPoint(_pin0);
          cv::Vec3d pin0_xyz( pt[0], pt[1], pt[2] );
          pt = _mesh->GetPoint(_pin1);
          cv::Vec3d pin1_xyz( pt[0], pt[1], pt[2] );
          cv::Vec3d sub = pin0_xyz - pin1_xyz;

          sub[0] = fabs(sub[0]);
          sub[1] = fabs(sub[1]);
          sub[2] = fabs(sub[2]);

          if ((sub[0] > sub[1]) && (sub[0] > sub[2])) {
            dirx = 0;
            diry = (sub[1] > sub[2]) ? 1 : 2;
          }
          else if ((sub[1] > sub[0]) && (sub[1] > sub[2])) {
            dirx = 1;
            diry = (sub[0] > sub[2]) ? 0 : 2;
          }
          else {
            dirx = 2;
            diry = (sub[0] > sub[1]) ? 0 : 1;
          }

          if (dirx == 2) {
            diru = 1;
            dirv = 0;
          }
          else {
            diru = 0;
            dirv = 1;
          }

          _vertInfo[_pin0].uv[diru] = pin0_xyz[dirx];
          _vertInfo[_pin0].uv[dirv] = pin0_xyz[diry];
          _vertInfo[_pin1].uv[diru] = pin1_xyz[dirx];
          _vertInfo[_pin1].uv[dirv] = pin1_xyz[diry];
        }
      }


    } // texturing
} // volcart