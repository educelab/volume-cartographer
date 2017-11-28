#include "vc/texturing/FlatteningAlgorithmBaseClass.hpp"

#include "vc/core/util/MeshMath.hpp"

using namespace volcart;
using namespace volcart::texturing;

namespace mm = volcart::meshmath;

UVMap FlatteningAlgorithmBaseClass::getUVMap()
{
    // Setup uvMap
    volcart::UVMap uvMap;

    // Get bounds
    auto bb = ITKMesh::BoundingBoxType::New();
    bb->SetPoints(output_->GetPoints());
    bb->ComputeBoundingBox();

    auto uMin = bb->GetBounds()[0];
    auto uMax = bb->GetBounds()[1];
    auto vMin = bb->GetBounds()[4];
    auto vMax = bb->GetBounds()[5];

    // Set the UV map ratio and get linear scale factor
    auto scale = sqrt(mm::SurfaceArea(mesh_) / mm::SurfaceArea(output_));
    auto aspectWidth = std::abs(uMax - uMin);
    auto aspectHeight = std::abs(vMax - vMin);
    uvMap.ratio(aspectWidth * scale, aspectHeight * scale);

    // Calculate uv coordinates
    cv::Vec2d uv;
    for (auto it = output_->GetPoints()->Begin();
         it != output_->GetPoints()->End(); ++it) {
        uv[0] = (it->Value()[0] - uMin) / (uMax - uMin);
        uv[1] = (it->Value()[2] - vMin) / (vMax - vMin);

        // Add the uv coordinates into our map at the point index specified
        uvMap.set(it->Index(), uv);
    }

    return uvMap;
}

// Reorient the UV map such that the Z-axis of the input is parallel to V-axis.
// The original orientation is approximated by a least-squares fit of the pts
// in UV space. Assumes that the UV coordinates are stored in output_
void FlatteningAlgorithmBaseClass::orient_uvs_()
{
    // Get the UV points + Z (from the original mesh)
    std::vector<cv::Vec3d> pts;
    for (unsigned id = 0; id < output_->GetNumberOfPoints(); id++) {
        auto uv = output_->GetPoint(id);
        pts.emplace_back(uv[0], uv[2], mesh_->GetPoint(id)[2]);
    }

    // Fit a 3D least-squares line
    cv::Vec6d line;
    cv::fitLine(pts, line, CV_DIST_L2, 0, 0.01, 0.01);

    // Calculate a rotation angle between the fitted line and the y vec
    cv::Vec2d fitVecUV;
    cv::normalize(cv::Vec2d{line[0], line[1]}, fitVecUV);
    cv::Vec2d downVecUV{0, 1};
    auto cos = fitVecUV.dot(downVecUV) / cv::norm(fitVecUV);
    auto angle = std::acos(cos);

    // If z-gradient is decreasing, rot. 180 deg s.t. z = 0 is at the top
    if (line[2] < 0) {
        angle += M_PI;
        cos = std::cos(angle);
    }

    auto sin = std::sin(angle);

    // Get counter-clockwise rotation matrix
    cv::Mat rotMat = cv::Mat::zeros(3, 3, CV_64F);
    rotMat.at<double>(0, 0) = cos;
    rotMat.at<double>(0, 1) = sin;
    rotMat.at<double>(1, 0) = -sin;
    rotMat.at<double>(1, 1) = cos;
    rotMat.at<double>(2, 2) = 1.0;

    // Rotate all of the points
    cv::transform(pts, pts, rotMat);

    // Apply to the UVs
    ITKMesh::PointType p;
    p[1] = 0.0;
    for (unsigned id = 0; id < output_->GetNumberOfPoints(); id++) {
        p[0] = pts.at(id)[0];
        p[2] = pts.at(id)[1];
        output_->SetPoint(id, p);
    }
}