#include "vc/texturing/AlignmentMarkerGenerator.hpp"

#include <exception>
#include <random>

#include <opencv2/imgproc.hpp>
#include <vtkIdList.h>
#include <vtkOBBTree.h>
#include <vtkPoints.h>

#include "vc/core/util/BarycentricCoordinates.hpp"
#include "vc/core/util/ImageConversion.hpp"
#include "vc/meshing/ITK2VTK.hpp"
#include "vc/meshing/UVMapToITKMesh.hpp"

using namespace volcart;
using namespace volcart::texturing;

namespace vcm = volcart::meshing;

// Convert HSV values to RGB
cv::Scalar HSVtoRGB(float h, float s, float v);

AlignmentMarkerGenerator::LineSegment::LineSegment(
    const cv::Vec3d& a, const cv::Vec3d& b, cv::Scalar c)
    : a{a}, b{b}, color{std::move(c)}
{
}

void AlignmentMarkerGenerator::setInputMeshes(
    std::vector<volcart::TexturedMesh> m)
{
    input_ = std::move(m);
}

void AlignmentMarkerGenerator::setLineSegments(std::vector<LineSegment> r)
{
    lineSegments_ = std::move(r);
}

void AlignmentMarkerGenerator::setMarkerRadius(int r) { markerRadius_ = r; }

void AlignmentMarkerGenerator::setMarkerUseRandomColor(bool b)
{
    markerRandomColor_ = b;
}

std::vector<cv::Mat> AlignmentMarkerGenerator::getMarkedImages() const
{
    return output_;
}

std::vector<cv::Mat> AlignmentMarkerGenerator::compute()
{
    // Setup output
    output_.clear();

    // Generate a color for each line segment
    if (markerRandomColor_) {
        std::random_device rnd;
        std::default_random_engine gen(rnd());
        std::uniform_real_distribution<float> distribution(0, 360);
        // Random hue, constant saturation and brightness
        for (auto& r : lineSegments_) {
            r.color = HSVtoRGB(distribution(gen), 1, 1);
        }
    }

    // For each mesh...
    vcm::UVMapToITKMesh uv2mesh;
    auto obb = vtkSmartPointer<vtkOBBTree>::New();
    for (const auto& m : input_) {
        // Convert image to 8bpc, 3-channel
        auto marked = QuantizeImage(m.img, CV_8U);
        marked = ColorConvertImage(marked, 3);
        auto w = marked.cols;
        auto h = marked.rows;

        // Convert UVMap -> mesh -> vtkMesh (for convenience)
        uv2mesh.setMesh(m.mesh);
        uv2mesh.setUVMap(m.uv);
        auto uvMesh = uv2mesh.compute();
        auto vtkUVMesh = vtkSmartPointer<vtkPolyData>::New();
        vcm::ITK2VTK(uvMesh, vtkUVMesh);

        // Convert input mesh to vtkPolydata for OBBTree
        auto vtkMesh = vtkSmartPointer<vtkPolyData>::New();
        vcm::ITK2VTK(m.mesh, vtkMesh);

        // Build obbtree for original mesh
        cv::Vec3d origin, b0, b1, b2;
        double size[3];
        obb->ComputeOBB(vtkMesh, origin.val, b0.val, b1.val, b2.val, size);
        obb->SetDataSet(vtkMesh);
        obb->BuildLocator();

        // For each line segment...
        for (const auto& seg : lineSegments_) {
            // Intersect seg with mesh
            auto iPts = vtkSmartPointer<vtkPoints>::New();
            auto iCells = vtkSmartPointer<vtkIdList>::New();
            int ret =
                obb->IntersectWithLine(seg.a.val, seg.b.val, iPts, iCells);

            // No intersections, skip this seg
            if (ret == 0) {
                continue;
            }

            // For each intersection...
            for (vtkIdType iID = 0; iID < iPts->GetNumberOfPoints(); iID++) {
                // Get the 3D point and the face vert positions
                cv::Vec3d pt_3D, v0_3D, v1_3D, v2_3D;
                iPts->GetPoint(iID, pt_3D.val);
                auto cellPts =
                    vtkMesh->GetCell(iCells->GetId(iID))->GetPoints();
                cellPts->GetPoint(0, v0_3D.val);
                cellPts->GetPoint(1, v1_3D.val);
                cellPts->GetPoint(2, v2_3D.val);

                // Get the 2D face vert positions
                cv::Vec3d pt_2D, v0_2D, v1_2D, v2_2D;
                cellPts = vtkUVMesh->GetCell(iCells->GetId(iID))->GetPoints();
                cellPts->GetPoint(0, v0_2D.val);
                cellPts->GetPoint(1, v1_2D.val);
                cellPts->GetPoint(2, v2_2D.val);

                // Convert 3D -> UV
                auto b = CartesianToBarycentric(pt_3D, v0_3D, v1_3D, v2_3D);
                pt_2D = BarycentricToCartesian(b, v0_2D, v1_2D, v2_2D);

                // Convert UV -> 2D
                cv::Point center(
                    static_cast<int>(pt_2D[0] * w),
                    static_cast<int>(pt_2D[2] * h));

                // Draw intersection point
                cv::circle(
                    marked, center, markerRadius_, seg.color, cv::FILLED);
            }
        }

        output_.push_back(marked);
    }

    return output_;
}

cv::Scalar HSVtoRGB(float h, float s, float v)
{
    cv::Mat hsv(1, 1, CV_32FC3, cv::Scalar(h, s, v));
    cv::Mat rgb;
    cv::cvtColor(hsv, rgb, cv::COLOR_HSV2BGR);

    return {255 * rgb.at<float>(2), 255 * rgb.at<float>(1),
            255 * rgb.at<float>(0)};
}