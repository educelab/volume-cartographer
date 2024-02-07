#include "vc/texturing/PPMGenerator.hpp"

#include <cstdint>
#include <exception>

#include <bvh/bvh.hpp>
#include <bvh/primitive_intersectors.hpp>
#include <bvh/ray.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/sweep_sah_builder.hpp>
#include <bvh/triangle.hpp>
#include <bvh/vector.hpp>
#include <opencv2/core.hpp>

#include "vc/core/util/BarycentricCoordinates.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/meshing/CalculateNormals.hpp"

using namespace volcart;
using namespace texturing;

namespace vcm = volcart::meshing;
namespace vct = volcart::texturing;

static constexpr std::uint8_t MASK_TRUE{255};

using Scalar = double;
using Vector3 = bvh::Vector3<Scalar>;
using Triangle = bvh::Triangle<Scalar>;
using Ray = bvh::Ray<Scalar>;
using Bvh = bvh::Bvh<Scalar>;
using Intersector = bvh::ClosestPrimitiveIntersector<Bvh, Triangle>;
using Traverser = bvh::SingleRayTraverser<Bvh>;

PPMGenerator::PPMGenerator(std::size_t h, std::size_t w) : width_{w}, height_{h}
{
}

void PPMGenerator::setMesh(const ITKMesh::Pointer& m) { inputMesh_ = m; }

void PPMGenerator::setUVMap(const UVMap::Pointer& u) { uvMap_ = u; }

// Parameters
void PPMGenerator::setDimensions(std::size_t h, std::size_t w)
{
    height_ = h;
    width_ = w;
}

void PPMGenerator::setShading(PPMGenerator::Shading s) { shading_ = s; }

auto PPMGenerator::getPPM() const -> PerPixelMap::Pointer { return ppm_; }

auto PPMGenerator::progressIterations() const -> std::size_t
{
    return width_ * height_;
}

// Compute
auto PPMGenerator::compute() -> PerPixelMap::Pointer
{
    if (inputMesh_.IsNull() || inputMesh_->GetNumberOfPoints() == 0 ||
        inputMesh_->GetNumberOfCells() == 0 || not uvMap_ || uvMap_->empty() ||
        width_ == 0 || height_ == 0) {
        const auto* msg = "Invalid input parameters";
        throw std::invalid_argument(msg);
    }

    // Generate normals
    if (shading_ == Shading::Smooth &&
        inputMesh_->GetPointData()->Size() != inputMesh_->GetNumberOfPoints()) {
        vcm::CalculateNormals normCalc(inputMesh_);
        workingMesh_ = normCalc.compute();
    } else {
        workingMesh_ = inputMesh_;
    }

    // Setup the output
    ppm_ = PerPixelMap::New(height_, width_);
    cv::Mat mask = cv::Mat::zeros(height_, width_, CV_8UC1);
    cv::Mat cellMap = cv::Mat(height_, width_, CV_32SC1);
    cellMap = cv::Scalar::all(-1);

    // Create BVH for mesh
    std::vector<Triangle> triangles;
    for (auto cell = workingMesh_->GetCells()->Begin();
         cell != workingMesh_->GetCells()->End(); ++cell) {
        // Get the vertex IDs
        auto a = cell->Value()->GetPointIdsContainer().GetElement(0);
        auto b = cell->Value()->GetPointIdsContainer().GetElement(1);
        auto c = cell->Value()->GetPointIdsContainer().GetElement(2);

        auto uvA = uvMap_->get(a);
        auto uvB = uvMap_->get(b);
        auto uvC = uvMap_->get(c);

        // Add the face to the BVH tree
        triangles.emplace_back(
            Vector3(uvA[0], uvA[1], 0), Vector3(uvB[0], uvB[1], 0),
            Vector3(uvC[0], uvC[1], 0));
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

    // Iterate over all of the pixels
    progressStarted();
    ITKCell::CellAutoPointer cell;
    for (const auto [y, x] : range2D(height_, width_)) {
        progressUpdated(y * width_ + x);
        // This pixel's uv coordinate
        cv::Vec3d uv{0, 0, 0};
        uv[0] = static_cast<double>(x) / static_cast<double>(width_ - 1);
        uv[1] = static_cast<double>(y) / static_cast<double>(height_ - 1);

        // Intersect a ray with the data structure
        Ray ray(Vector3(uv[0], uv[1], 0), Vector3(uv[0], uv[1], 1.0), 0.0, 1.0);
        auto hit = traverser.traverse(ray, intersector);
        if (not hit) {
            continue;
        }

        // Cell info
        auto cellId = hit->primitive_index;
        cell.TakeOwnership(new ITKTriangle);
        workingMesh_->GetCell(cellId, cell);
        auto a = cell->GetPointIdsContainer().GetElement(0);
        auto b = cell->GetPointIdsContainer().GetElement(1);
        auto c = cell->GetPointIdsContainer().GetElement(2);

        // Get the 2D and 3D pts
        std::vector<cv::Vec3d> uvPts;
        std::vector<cv::Vec3d> xyzPts;
        for (const auto& idx : {a, b, c}) {
            auto uvPt = uvMap_->get(idx);
            auto xyzPt = workingMesh_->GetPoint(idx);
            uvPts.emplace_back(uvPt[0], uvPt[1], 0.0);
            xyzPts.emplace_back(xyzPt[0], xyzPt[1], xyzPt[2]);
        }

        // Find the xyz coordinate of the original point
        auto baryCoord =
            CartesianToBarycentric(uv, uvPts[0], uvPts[1], uvPts[2]);
        auto xyz =
            BarycentricToCartesian(baryCoord, xyzPts[0], xyzPts[1], xyzPts[2]);

        // Get this corresponding normal
        cv::Vec3d xyzNorm;
        if (shading_ == Shading::Flat) {
            auto v1v0 = xyzPts[1] - xyzPts[0];
            auto v2v0 = xyzPts[2] - xyzPts[0];
            xyzNorm = cv::normalize(v1v0.cross(v2v0));
        } else {
            ITKPixel nA;
            ITKPixel nB;
            ITKPixel nC;
            auto found = workingMesh_->GetPointData(a, &nA);
            found &= workingMesh_->GetPointData(b, &nB);
            found &= workingMesh_->GetPointData(c, &nC);
            if (not found) {
                throw std::runtime_error(
                    "Performing smooth shading but missing vertex normal");
            }
            xyzNorm = BarycentricNormalInterpolation(
                baryCoord, {nA[0], nA[1], nA[2]}, {nB[0], nB[1], nB[2]},
                {nC[0], nC[1], nC[2]});
        }

        // Assign the cell index to the cell map
        auto intX = static_cast<int>(x);
        auto intY = static_cast<int>(y);
        cellMap.at<std::int32_t>(intY, intX) = cellId;

        // Assign the intensity value at the UV position
        mask.at<std::uint8_t>(intY, intX) = MASK_TRUE;

        // Assign 3D position to the lookup map
        ppm_->getMapping(y, x) = cv::Vec6d(
            xyz(0), xyz(1), xyz(2), xyzNorm(0), xyzNorm(1), xyzNorm(2));
    }
    progressComplete();

    // Finish setting up the output
    ppm_->setMask(mask);
    ppm_->setCellMap(cellMap);

    return ppm_;
}

auto vct::GenerateCellMap(
    const ITKMesh::Pointer& mesh,
    const UVMap::Pointer& uvMap,
    std::size_t height,
    std::size_t width) -> cv::Mat
{

    auto cellMap = cv::Mat(height, width, CV_32SC1);
    cellMap = cv::Scalar::all(-1);

    // Create BVH for mesh
    std::vector<Triangle> triangles;
    for (auto cell = mesh->GetCells()->Begin(); cell != mesh->GetCells()->End();
         ++cell) {
        // Get the vertex IDs
        auto a = cell->Value()->GetPointIdsContainer().GetElement(0);
        auto b = cell->Value()->GetPointIdsContainer().GetElement(1);
        auto c = cell->Value()->GetPointIdsContainer().GetElement(2);

        auto uvA = uvMap->get(a);
        auto uvB = uvMap->get(b);
        auto uvC = uvMap->get(c);

        // Add the face to the BVH tree
        triangles.emplace_back(
            Vector3(uvA[0], uvA[1], 0), Vector3(uvB[0], uvB[1], 0),
            Vector3(uvC[0], uvC[1], 0));
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

    ITKCell::CellAutoPointer cell;
    for (const auto [y, x] : range2D(height, width)) {
        // This pixel's uv coordinate
        cv::Vec3d uv{0, 0, 0};
        uv[0] = static_cast<double>(x) / static_cast<double>(width - 1);
        uv[1] = static_cast<double>(y) / static_cast<double>(height - 1);

        // Intersect a ray with the data structure
        Ray ray(Vector3(uv[0], uv[1], 0), Vector3(uv[0], uv[1], 1.0), 0.0, 1.0);
        auto hit = traverser.traverse(ray, intersector);
        if (not hit) {
            continue;
        }

        // Assign the cell index to the cell map
        auto intX = static_cast<int>(x);
        auto intY = static_cast<int>(y);
        cellMap.at<std::int32_t>(intY, intX) =
            static_cast<int>(hit->primitive_index);
    }

    return cellMap;
}
