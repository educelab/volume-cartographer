#include "vc/texturing/ProjectMesh.hpp"

#include <cstdint>
#include <utility>

#include <bvh/bvh.hpp>
#include <bvh/primitive_intersectors.hpp>
#include <bvh/ray.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/sweep_sah_builder.hpp>
#include <bvh/triangle.hpp>
#include <bvh/vector.hpp>
#include <vtkOBBTree.h>
#include <vtkPointData.h>

#include "vc/core/util/BarycentricCoordinates.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/meshing/ITK2VTK.hpp"

static constexpr std::uint8_t MASK_TRUE{255};

using Scalar = double;
using Vector3 = bvh::Vector3<Scalar>;
using Triangle = bvh::Triangle<Scalar>;
using Ray = bvh::Ray<Scalar>;
using Bvh = bvh::Bvh<Scalar>;
using Intersector = bvh::ClosestPrimitiveIntersector<Bvh, Triangle>;
using Traverser = bvh::SingleRayTraverser<Bvh>;

namespace vc = volcart;
namespace vcm = vc::meshing;
namespace vct = vc::texturing;

void vct::ProjectMesh::setMesh(const ITKMesh::Pointer& inputMesh)
{
    inputMesh_ = inputMesh;
}

void vct::ProjectMesh::setTransform(CompositeTransform::Pointer transform)
{
    tfm_ = std::move(transform);
}

void vct::ProjectMesh::setUseInverseTransform(bool useInverse)
{
    useInverse_ = useInverse;
}

void vct::ProjectMesh::setSampleMode(SampleMode mode) { mode_ = mode; }

void vct::ProjectMesh::setTextureDimensions(int width, int height)
{
    textureWidth_ = width;
    textureHeight_ = height;
}

void vct::ProjectMesh::setPPMDimensions(int width, int height)
{
    ppmWidth_ = width;
    ppmHeight_ = height;
}

void vct::ProjectMesh::setSampleRate(double rate)
{
    sampleRateX_ = sampleRateY_ = rate;
}

void vct::ProjectMesh::setUseFirstIntersection(bool useFirstIntersection)
{
    useFirstIntersection_ = useFirstIntersection;
}

auto vct::ProjectMesh::compute() -> vc::PerPixelMap
{
    if (!inputMesh_) {
        throw std::runtime_error("Empty input mesh");
    }

    if (useInverse_) {
        if (!tfm_->GetInverseTransform()) {
            throw std::runtime_error(
                "Unable to invert transform, your composite transform may have "
                "b-spline transforms");
        }
    }

    // Convert input mesh to VTK
    auto vtkMesh = vtkSmartPointer<vtkPolyData>::New();
    vcm::ITK2VTK(inputMesh_, vtkMesh);

    // Computes the OBB and returns the 3 axes relative to the box
    cv::Vec3d origin, b0, b1, b2;
    double size[3];
    auto obbTree = vtkSmartPointer<vtkOBBTree>::New();
    obbTree->ComputeOBB(vtkMesh, origin.val, b0.val, b1.val, b2.val, size);
    obbTree->SetDataSet(vtkMesh);
    obbTree->BuildLocator();

    // Set the marching parameters
    if (mode_ == SampleMode::Rate) {
        ppmWidth_ = static_cast<int>(std::ceil(cv::norm(b0) / sampleRateX_));
        ppmHeight_ = static_cast<int>(std::ceil(cv::norm(b1) / sampleRateY_));
    } else if (mode_ == SampleMode::Dimensions) {
        sampleRateX_ = cv::norm(b0) / ppmWidth_;
        sampleRateY_ = cv::norm(b1) / ppmHeight_;
    }

    outputPPM_.setDimensions(ppmHeight_, ppmWidth_);
    cv::Mat mask = cv::Mat::zeros(ppmHeight_, ppmWidth_, CV_8UC1);
    cv::Mat cellMap = cv::Mat(ppmHeight_, ppmWidth_, CV_32SC1);
    cellMap = cv::Scalar::all(-1);

    // Normalize the length
    auto normedX = b0 / cv::norm(b0);
    auto normedY = b1 / cv::norm(b1);
    if (cv::norm(b2) < 1.0) {
        b2 = normedY.cross(normedX);
    }

    // Create BVH for mesh
    std::vector<Triangle> triangles;
    for (auto cell = inputMesh_->GetCells()->Begin();
         cell != inputMesh_->GetCells()->End(); ++cell) {
        auto aIdx = cell.Value()->GetPointIdsContainer()[0];
        auto bIdx = cell.Value()->GetPointIdsContainer()[1];
        auto cIdx = cell.Value()->GetPointIdsContainer()[2];

        auto a = inputMesh_->GetPoint(aIdx);
        auto b = inputMesh_->GetPoint(bIdx);
        auto c = inputMesh_->GetPoint(cIdx);

        // Add the face to the BVH tree
        triangles.emplace_back(
            Vector3(a[0], a[1], a[2]), Vector3(b[0], b[1], b[2]),
            Vector3(c[0], c[1], c[2]));
    }
    Bvh bvh;
    bvh::SweepSahBuilder<Bvh> builder(bvh);
    auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(
        triangles.data(), triangles.size());
    auto meshBBox =
        bvh::compute_bounding_boxes_union(bboxes.get(), triangles.size());
    builder.build(meshBBox, bboxes.get(), centers.get(), triangles.size());
    Intersector intersector(bvh, triangles.data());
    Traverser traverser(bvh);

    auto tfm = tfm_;
    if (useInverse_) {
        tfm = dynamic_cast<CompositeTransform*>(
            tfm_->GetInverseTransform().GetPointer());
    }

    // Loop over every pixel of the image
    for (const auto uv : vc::range2D(ppmHeight_, ppmWidth_)) {
        const auto& u = uv.second;
        const auto& v = uv.first;

        cv::Vec3d uOffset;
        cv::Vec3d vOffset;
        if (tfm) {
            Point p;
            p[0] = u;
            p[1] = v;
            auto pT = tfm->TransformPoint(p);
            uOffset = pT[0] / textureWidth_ * b0;
            vOffset = pT[1] / textureHeight_ * b1;
        } else {
            // Convert pixel position to offset in mesh's XY space
            uOffset = u * sampleRateX_ * normedX;
            vOffset = v * sampleRateY_ * normedY;
        }

        auto a0 = origin + uOffset + vOffset;
        auto a1 = b2;
        if (not useFirstIntersection_) {
            a0 += b2 * cv::norm(b2);
            a1 *= -1;
        }

        // Intersect a ray with the data structure
        Vector3 start(a0[0], a0[1], a0[2]);
        Vector3 dir(a1[0], a1[1], a1[2]);
        Ray ray(start, dir, 0.0, cv::norm(b2) * 2);
        auto hit = traverser.traverse(ray, intersector);
        if (not hit) {
            continue;
        }

        // Cell info
        auto cellId = static_cast<vtkIdType>(hit->primitive_index);
        auto pointIds = vtkSmartPointer<vtkIdList>::New();
        vtkMesh->GetCellPoints(cellId, pointIds);

        // Make sure we only have three vertices
        assert(pointIds->GetNumberOfIds() == 3);
        auto v_id0 = pointIds->GetId(0);
        auto v_id1 = pointIds->GetId(1);
        auto v_id2 = pointIds->GetId(2);

        // Get the 3D positions of each vertex
        cv::Vec3d A{vtkMesh->GetPoint(v_id0)};
        cv::Vec3d B{vtkMesh->GetPoint(v_id1)};
        cv::Vec3d C{vtkMesh->GetPoint(v_id2)};

        // Intersection point UV coords
        auto inter = hit->intersection;
        cv::Vec3d bCoord{inter.u, inter.v, 1 - inter.u - inter.v};

        // Get the 3D position of the intersection pt
        auto xyz = BarycentricToCartesian(bCoord, A, B, C);

        // Interpolate the vertex normal for this point
        cv::Vec3d n0, n1, n2;
        auto bary = CartesianToBarycentric(xyz, A, B, C);
        vtkMesh->GetPointData()->GetNormals()->GetTuple(v_id0, n0.val);
        vtkMesh->GetPointData()->GetNormals()->GetTuple(v_id1, n1.val);
        vtkMesh->GetPointData()->GetNormals()->GetTuple(v_id2, n2.val);
        auto xyzNorm = BarycentricNormalInterpolation(bary, n0, n1, n2);

        // Assign the cell index to the cell map
        auto intU = static_cast<int>(u);
        auto intV = static_cast<int>(v);
        cellMap.at<std::int32_t>(intV, intU) = static_cast<int>(cellId);

        // Assign 3D position to the lookup map and update the mask
        outputPPM_(v, u) = cv::Vec6d{xyz(0),     xyz(1),     xyz(2),
                                     xyzNorm(0), xyzNorm(1), xyzNorm(2)};
        mask.at<std::uint8_t>(v, u) = MASK_TRUE;
    }

    outputPPM_.setMask(mask);
    outputPPM_.setCellMap(cellMap);
    return outputPPM_;
}
