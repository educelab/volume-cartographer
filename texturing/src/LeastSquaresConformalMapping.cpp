#include "texturing/LeastSquaresConformalMapping.h"
#include "meshing/deepCopy.h"
#include <igl/lscm.h>
#include <igl/boundary_loop.h>
#include <igl/doublearea.h>
#include <opencv2/opencv.hpp>

using namespace volcart::texturing;

///// Constructors /////
LeastSquaresConformalMapping::LeastSquaresConformalMapping( VC_MeshType::Pointer input ) : _mesh(input) {
  _fillEigenMatrices();
};

///// Input/Output /////
// Set input mesh
void LeastSquaresConformalMapping::setMesh( VC_MeshType::Pointer input ) {
  _emptyEigenMatrices();
  _mesh = input;
  _fillEigenMatrices();
}

// Get output as mesh
VC_MeshType::Pointer LeastSquaresConformalMapping::getMesh() {
  VC_MeshType::Pointer output = VC_MeshType::New();
  volcart::meshing::deepCopy( _mesh, output );

  // Update the point positions
  VC_PointType p;
  for ( unsigned long i = 0; i < _vertices_UV.rows(); ++i ) {
    p[0] = _vertices_UV(i, 0);
    p[1] = 0;
    p[2] = _vertices_UV(i, 1);
    output->SetPoint(i, p);
  }

  // To-do: Recompute normals

  return output;
}

// Get UV Map created from flattened object
volcart::UVMap LeastSquaresConformalMapping::getUVMap() {

  // Setup uvMap
  volcart::UVMap uvMap;

  double min_u = std::numeric_limits<double>::max();
  double max_u = std::numeric_limits<double>::min();
  double min_v = std::numeric_limits<double>::max();
  double max_v = std::numeric_limits<double>::min();

  for ( int i = 0; i < _vertices_UV.rows(); ++i ) {
    if ( _vertices_UV(i, 0) < min_u ) min_u = _vertices_UV(i, 0);
    if ( _vertices_UV(i, 0) > max_u ) max_u = _vertices_UV(i, 0);

    if ( _vertices_UV(i, 1) < min_v ) min_v = _vertices_UV(i, 1);
    if ( _vertices_UV(i, 1) > max_v ) max_v = _vertices_UV(i, 1);
  }

  // Scale width and height back to volume coordinates
  double scaleFactor = std::sqrt(_startingArea / _area( _vertices_UV, _faces ));
  double aspect_width = std::abs(max_u - min_u) * scaleFactor;
  double aspect_height = std::abs(max_v - min_v) * scaleFactor;
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

///// Processing /////
// Compute the parameterization
void LeastSquaresConformalMapping::compute() {

  // Fix two points on the boundary
  Eigen::VectorXi bnd, b(2,1);
  igl::boundary_loop( _faces, bnd );
  b(0) = bnd(0);
  b(1) = bnd(round(bnd.size()/2));
  Eigen::MatrixXd bc(2,2);
  bc << 0,0,1,1;

  // LSCM parametrization
  igl::lscm( _vertices, _faces, b, bc, _vertices_UV );

  // Find the line of best fit through the flattened points
  // Use this line to try to straighten the textures
  // Note: This will only work with segmentations that are wider than they are long
  std::vector<cv::Point2f> points;
  cv::Vec4f line;
  for ( int i = 0; i < _vertices_UV.rows(); ++i ) {
    cv::Point2d p;
    p.x = _vertices_UV(i, 0);
    p.y = _vertices_UV(i, 1);
    points.push_back(p);
  }
  cv::fitLine( points, line, CV_DIST_L2, 0, 0.01, 0.01);
  Eigen::Rotation2Dd rot( std::atan(line(1)/line(0)) );
  _vertices_UV *= rot.matrix();
}

///// Utilities /////
// Fill the data structures with the mesh
void LeastSquaresConformalMapping::_fillEigenMatrices() {

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

  // Set the starting area for later comparison
  _startingArea = _area( _vertices, _faces);
}

// Empty the data structures
void LeastSquaresConformalMapping::_emptyEigenMatrices() {
  _vertices    = Eigen::MatrixXd();
  _faces       = Eigen::MatrixXi();
  _vertices_UV = Eigen::MatrixXd();
}

// Calculate surface area of meshes
double LeastSquaresConformalMapping::_area(const Eigen::MatrixXd& v, const Eigen::MatrixXi& f) {
  Eigen::VectorXd area;
  igl::doublearea(v, f, area);
  area = area.array() / 2;

  // doublearea returns array of signed areas
  double a = 0.0;
  for( auto i = 0; i < area.size(); ++i ) {
    a += std::abs( area[i] );
  }

  return a;
}
