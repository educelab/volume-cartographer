#include "vc/texturing/FlatteningAlgorithm.hpp"

#include "vc/core/util/FloatComparison.hpp"
#include "vc/core/util/MeshMath.hpp"

using namespace volcart;
using namespace volcart::texturing;

namespace mm = volcart::meshmath;

UVMap FlatteningAlgorithm::getUVMap()
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
void FlatteningAlgorithm::orient_uvs_()
{
    // Get the UV points + Z (from the original mesh)
    std::vector<cv::Vec3d> pts;
    for (unsigned id = 0; id < output_->GetNumberOfPoints(); id++) {
        auto uv = output_->GetPoint(id);
        pts.emplace_back(uv[0], uv[2], mesh_->GetPoint(id)[2]);
    }

    // Fit a 3D least-squares line
    cv::Vec6d line;
    cv::fitLine(pts, line, cv::DIST_L2, 0, 0.01, 0.01);

    // Calculate a rotation angle between the fitted line and the y vec
    cv::Point2d center(line[3], line[4]);
    cv::Vec2d lineVec;
    cv::normalize(cv::Vec2d{line[0], line[1]}, lineVec);
    cv::Vec2d yAxis{0, 1};
    auto cos = lineVec.dot(yAxis);
    auto angle = std::acos(cos);

    // If z-gradient is decreasing, rot. 180 deg s.t. z = 0 is at the top
    if (line[2] < 0) {
        angle += M_PI;
        cos = std::cos(angle);
    }
    auto sin = std::sin(angle);

    // Clamp to zero
    if (AlmostEqual<double>(cos, 0)) {
        cos = 0;
    }
    if (AlmostEqual<double>(sin, 0)) {
        sin = 0;
    }

    // Get translation matrix
    cv::Mat transMat = cv::Mat::eye(3, 3, CV_64F);
    transMat.at<double>(0, 2) = -center.x;
    transMat.at<double>(1, 2) = -center.y;

    // Get counter-clockwise rotation matrix
    cv::Mat rotMat = cv::Mat::eye(3, 3, CV_64F);
    rotMat.at<double>(0, 0) = cos;
    rotMat.at<double>(0, 1) = sin;
    rotMat.at<double>(1, 0) = -sin;
    rotMat.at<double>(1, 1) = cos;

    // Composite
    cv::Mat composite = rotMat * transMat;

    // Translate and rotate all of the points
    // Note: pts vector is column-major, so transpose the transform matrix
    cv::transform(pts, pts, composite.t());

    // Apply to the UVs
    ITKMesh::PointType p;
    p[1] = 0.0;
    for (unsigned id = 0; id < output_->GetNumberOfPoints(); id++) {
        p[0] = pts.at(id)[0];
        p[2] = pts.at(id)[1];
        output_->SetPoint(id, p);
    }
}
