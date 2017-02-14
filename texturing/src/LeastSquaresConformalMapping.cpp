#include <cmath>

#include <igl/boundary_loop.h>
#include <igl/doublearea.h>
#include <igl/lscm.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "vc/meshing/DeepCopy.hpp"
#include "texturing/LeastSquaresConformalMapping.hpp"

using namespace volcart;
using namespace volcart::texturing;

///// Constructors /////
LeastSquaresConformalMapping::LeastSquaresConformalMapping(
    ITKMesh::Pointer input)
    : mesh_(input)
{
    fill_eigen_matrices_();
}

///// Input/Output /////
// Set input mesh
void LeastSquaresConformalMapping::setMesh(const ITKMesh::Pointer& input)
{
    empty_eigen_matrices_();
    mesh_ = input;
    fill_eigen_matrices_();
}

// Get output as mesh
ITKMesh::Pointer LeastSquaresConformalMapping::getMesh()
{
    ITKMesh::Pointer output = ITKMesh::New();
    volcart::meshing::DeepCopy(mesh_, output);

    // Update the point positions
    ITKPoint p;
    for (int64_t i = 0; i < verticesUV_.rows(); ++i) {
        p[0] = verticesUV_(i, 0);
        p[1] = 0;
        p[2] = verticesUV_(i, 1);
        output->SetPoint(i, p);
    }

    // To-do #191
    return output;
}

// Get UV Map created from flattened object
volcart::UVMap LeastSquaresConformalMapping::getUVMap()
{

    // Setup uvMap
    volcart::UVMap uvMap;
    uvMap.origin(VC_ORIGIN_BOTTOM_LEFT);

    double umin = std::numeric_limits<double>::max();
    double umax = std::numeric_limits<double>::min();
    double vmin = std::numeric_limits<double>::max();
    double vmax = std::numeric_limits<double>::min();

    for (int i = 0; i < verticesUV_.rows(); ++i) {
        if (verticesUV_(i, 0) < umin) {
            umin = verticesUV_(i, 0);
        }
        if (verticesUV_(i, 0) > umax) {
            umax = verticesUV_(i, 0);
        }
        if (verticesUV_(i, 1) < vmin) {
            vmin = verticesUV_(i, 1);
        }
        if (verticesUV_(i, 1) > vmax) {
            vmax = verticesUV_(i, 1);
        }
    }

    // Scale width and height back to volume coordinates
    double scaleFactor = std::sqrt(startingArea_ / area_(verticesUV_, faces_));
    double aspectWidth = std::abs(umax - umin) * scaleFactor;
    double aspectHeight = std::abs(vmax - vmin) * scaleFactor;
    uvMap.ratio(aspectWidth, aspectHeight);

    // Calculate uv coordinates
    double u, v;
    for (int i = 0; i < verticesUV_.rows(); ++i) {
        u = (verticesUV_(i, 0) - umin) / (umax - umin);
        v = (verticesUV_(i, 1) - vmin) / (vmax - vmin);
        cv::Vec2d uv(u, v);

        // Add the uv coordinates into our map at the point index specified
        uvMap.set(i, uv);
    }
    uvMap.origin(VC_ORIGIN_TOP_LEFT);
    return uvMap;
}

///// Processing /////
// Compute the parameterization
void LeastSquaresConformalMapping::compute()
{

    // Fix two points on the boundary
    Eigen::VectorXi bnd, b(2, 1);
    igl::boundary_loop(faces_, bnd);
    b(0) = bnd(0);
    b(1) = bnd(std::lround(bnd.size() / 2));
    Eigen::MatrixXd bc(2, 2);
    bc << 0, 0, 1, 1;

    // LSCM parametrization
    igl::lscm(vertices_, faces_, b, bc, verticesUV_);

    // Find the line of best fit through the flattened points
    // Use this line to try to straighten the textures
    // Note: This will only work with segmentations that are wider than they are
    // long
    std::vector<cv::Point2f> points;
    cv::Vec4f line;
    for (int i = 0; i < verticesUV_.rows(); ++i) {
        cv::Point2d p;
        p.x = verticesUV_(i, 0);
        p.y = verticesUV_(i, 1);
        points.push_back(p);
    }
    cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);
    Eigen::Rotation2Dd rot(std::atan(line(1) / line(0)));
    verticesUV_ *= rot.matrix();
}

///// Utilities /////
// Fill the data structures with the mesh
void LeastSquaresConformalMapping::fill_eigen_matrices_()
{

    // Vertices
    vertices_.resize(mesh_->GetNumberOfPoints(), 3);
    for (ITKPointIterator point = mesh_->GetPoints()->Begin();
         point != mesh_->GetPoints()->End(); ++point) {
        vertices_(point->Index(), 0) = point->Value()[0];
        vertices_(point->Index(), 1) = point->Value()[1];
        vertices_(point->Index(), 2) = point->Value()[2];
    }

    // Faces
    faces_.resize(mesh_->GetNumberOfCells(), 3);
    for (ITKCellIterator cell = mesh_->GetCells()->Begin();
         cell != mesh_->GetCells()->End(); ++cell) {

        int i = 0;
        for (ITKPointInCellIterator point = cell.Value()->PointIdsBegin();
             point != cell.Value()->PointIdsEnd(); ++point) {
            faces_(cell->Index(), i) = *point;
            ++i;
        }
    }

    // Set the starting area for later comparison
    startingArea_ = area_(vertices_, faces_);
}

// Empty the data structures
void LeastSquaresConformalMapping::empty_eigen_matrices_()
{
    vertices_ = Eigen::MatrixXd();
    faces_ = Eigen::MatrixXi();
    verticesUV_ = Eigen::MatrixXd();
}

// Calculate surface area of meshes
double LeastSquaresConformalMapping::area_(
    const Eigen::MatrixXd& v, const Eigen::MatrixXi& f)
{
    Eigen::VectorXd area;
    igl::doublearea(v, f, area);
    area = area.array() / 2;

    // doublearea returns array of signed areas
    double a = 0.0;
    for (int i = 0; i < area.size(); ++i) {
        a += std::abs(area[i]);
    }

    return a;
}
