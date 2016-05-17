#include "lscm.h"

namespace volcart {
    namespace texturing {

        // Constructor
        lscm::lscm( VC_MeshType::Pointer input ) : _mesh(input) {
          _fillEigenMatrices();
        };

        // Compute
        void lscm::compute() {

          // Fix two points on the boundary
          Eigen::VectorXi bnd, b(2,1);
          igl::boundary_loop( _faces, bnd );
          b(0) = bnd(0);
          b(1) = bnd(round(bnd.size()/2));
          Eigen::MatrixXd bc(2,2);
          bc<<0,0,1,0;

          // LSCM parametrization
          igl::lscm( _vertices, _faces, b, bc, _vertices_UV );
        }

        // Get output as mesh
        VC_MeshType::Pointer lscm::getMesh() {
          VC_MeshType::Pointer output = VC_MeshType::New();
          volcart::meshing::deepCopy( _mesh, output );

          // Update the point positions
          VC_PointType p;
          for ( int i = 0; i < _vertices_UV.rows(); ++i ) {
            p[0] = _vertices_UV(i, 0);
            p[1] = 0;
            p[2] = _vertices_UV(i, 1);
            output->SetPoint(i, p);
          }

          // To-do: Recompute normals

          return output;
        }

        // Get UV Map created from flattened object
        volcart::UVMap lscm::getUVMap() {

          // Setup uvMap
          volcart::UVMap uvMap;
          uvMap.origin( VC_ORIGIN_BOTTOM_LEFT ); // To-Do: Need to test this.

          double min_u = 1;
          double max_u = 0;
          double min_v = 1;
          double max_v = 0;

          for ( int i = 0; i < _vertices_UV.rows(); ++i ) {
            if ( _vertices_UV(i, 0) < min_u ) min_u = _vertices_UV(i, 0);
            if ( _vertices_UV(i, 0) > max_u ) max_u = _vertices_UV(i, 0);

            if ( _vertices_UV(i, 1) < min_v ) min_v = _vertices_UV(i, 1);
            if ( _vertices_UV(i, 1) > max_v ) max_v = _vertices_UV(i, 1);
          }

          // Scale width and height back to volume coordinates
          double aspect_width = std::abs(max_u - min_u);
          double aspect_height = std::abs(max_v - min_v);
          double aspect = aspect_width / aspect_height;
          uvMap.ratio(aspect_width, aspect_height);

          // Calculate uv coordinates
          double u, v;
          for ( int i = 0; i < _vertices_UV.rows(); ++i ) {
            u = ( _vertices_UV(i, 0) - min_u ) / (max_u - min_u);
            v = ( _vertices_UV(i, 1) - min_v ) / (max_v - min_v);
            cv::Vec2d uv( u, v );

            // Add the uv coordinates into our map at the point index specified
            uvMap.set(i, uv);
          }

          return uvMap;
        }

        // Fill the data structures
        void lscm::_fillEigenMatrices() {

          // Vertices
          _vertices.resize( _mesh->GetNumberOfPoints(), 3);
          for ( VC_PointsInMeshIterator point = _mesh->GetPoints()->Begin(); point != _mesh->GetPoints()->End(); ++point ) {
            _vertices( point->Index(), 0 ) = point->Value()[0];
            _vertices( point->Index(), 1 ) = point->Value()[1];
            _vertices( point->Index(), 2 ) = point->Value()[2];
          }

          // Faces
          _faces.resize( _mesh->GetNumberOfCells(), 3);
          for ( VC_CellIterator cell = _mesh->GetCells()->Begin(); cell != _mesh->GetCells()->End(); ++cell ) {

            int i = 0;
            for ( VC_PointsInCellIterator point = cell.Value()->PointIdsBegin(); point != cell.Value()->PointIdsEnd(); ++point ) {
              _faces(cell->Index(), i) = *point;
              ++i;
            }
          }
        }

    }
}