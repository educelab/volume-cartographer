#include <stdexcept>

#include "vc/core/util/BarycentricCoordinates.hpp"
#include "vc/meshing/CalculateNormals.hpp"
#include "vc/meshing/DeepCopy.hpp"
#include "vc/texturing/PPMGenerator.hpp"

using namespace volcart;
using namespace texturing;

namespace vcm = volcart::meshing;

static constexpr uint8_t MASK_TRUE{255};

PPMGenerator::PPMGenerator(size_t h, size_t w) : width_{w}, height_{h} {}

void PPMGenerator::setMesh(const ITKMesh::Pointer& m) { inputMesh_ = m; }

void PPMGenerator::setUVMap(const UVMap& u) { uvMap_ = u; }

// Parameters
void PPMGenerator::setDimensions(size_t h, size_t w)
{
    height_ = h;
    width_ = w;
}

void PPMGenerator::setShading(PPMGenerator::Shading s) { shading_ = s; }

PerPixelMap PPMGenerator::getPPM() const { return ppm_; }

cv::Mat PPMGenerator::getCellMap() const { return cellMap_; }

size_t PPMGenerator::progressIterations() const { return width_ * height_; }

// Compute
PerPixelMap PPMGenerator::compute()
{
    if (inputMesh_.IsNull() || inputMesh_->GetNumberOfPoints() == 0 ||
        inputMesh_->GetNumberOfCells() == 0 || uvMap_.empty() || width_ == 0 ||
        height_ == 0) {
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

    // Make sure the storage vectors are clean
    centroidMesh_ = ITKMesh::New();
    cellInformation_.clear();

    generate_centroid_mesh_();
    generate_ppm_();

    return ppm_;
}

// Generate the centroid mesh and other temporary data structures
void PPMGenerator::generate_centroid_mesh_()
{
    ITKPoint centroid;
    CellInfo info;
    for (auto cell = workingMesh_->GetCells()->Begin();
         cell != workingMesh_->GetCells()->End(); ++cell) {
        info.reset();

        cv::Vec3d twoD;
        cv::Vec3d threeD;
        cv::Vec3d normal;
        for (auto* point = cell->Value()->PointIdsBegin();
             point != cell->Value()->PointIdsEnd(); ++point) {
            auto pointID = *point;

            twoD[0] = uvMap_.get(pointID)[0];
            twoD[1] = uvMap_.get(pointID)[1];
            twoD[2] = 0.0;
            info.pts2D.push_back(twoD);

            threeD[0] = workingMesh_->GetPoint(pointID)[0];
            threeD[1] = workingMesh_->GetPoint(pointID)[1];
            threeD[2] = workingMesh_->GetPoint(pointID)[2];
            info.pts3D.push_back(threeD);

            // Get the vertex normals for this cell
            ITKPixel n;
            auto found = workingMesh_->GetPointData(pointID, &n);
            if (found) {
                info.normals.emplace_back(n[0], n[1], n[2]);
            }
        }

        // Calculate the cell centroid and maximum centroid-vertex distance
        info.centroid = ((info.pts2D[0] + info.pts2D[1] + info.pts2D[2]) / 3);
        centroid = info.centroid.val;
        for (const auto& v : info.pts2D) {
            kdMaxDist_ = std::max(kdMaxDist_, cv::norm(v - info.centroid));
        }

        // Get the flat surface normal for this cell
        if (shading_ == Shading::Flat) {
            info.normals.clear();
            auto v1v0 = info.pts3D[1] - info.pts3D[0];
            auto v2v0 = info.pts3D[2] - info.pts3D[0];
            info.normals.emplace_back(cv::normalize(v1v0.cross(v2v0)));
        }

        cellInformation_.push_back(info);
        centroidMesh_->SetPoint(cell.Index(), centroid);
    }
}

void PPMGenerator::generate_ppm_()
{
    // Setup the output
    ppm_ = PerPixelMap(height_, width_);
    mask_ = cv::Mat::zeros(height_, width_, CV_8UC1);
    cellMap_ = cv::Mat(height_, width_, CV_32SC1);
    cellMap_ = cv::Scalar::all(-1);

    // Setup the search tree
    kdTree_ = ITKPointsLocator::New();
    kdTree_->SetPoints(centroidMesh_->GetPoints());
    kdTree_->Initialize();

    // Iterate over all of the pixels
    bool hintValid{false};
    size_t lastCell{0};
    progressStarted();
    for (size_t y = 0; y < height_; ++y) {
        for (size_t x = 0; x < width_; ++x) {
            progressUpdated(y * width_ + x);
            find_cell_(x, y, hintValid, lastCell);
        }
    }
    progressComplete();

    // Finish setting up the output
    ppm_.setUVMap(uvMap_);
    ppm_.setMask(mask_);
    ppm_.setCellMap(cellMap_);
}

void PPMGenerator::find_cell_(
    size_t x, size_t y, bool& useHint, size_t& cellHint)
{
    // This pixel's uv coordinate
    cv::Vec3d uv{0, 0, 0};
    uv[0] = static_cast<double>(x) / static_cast<double>(width_ - 1);
    uv[1] = static_cast<double>(y) / static_cast<double>(height_ - 1);

    // Whether we've found a cell
    CellInfo info;
    cv::Vec3d baryCoord{0, 0, 0};

    // Use the last cell as a hint if it's within the distance range
    // Assumes norm is less expensive than barycentric test
    bool cellFound{false};
    if (useHint) {
        info = cellInformation_[cellHint];
        if (cv::norm(uv - info.centroid) <= kdMaxDist_) {
            baryCoord = CartesianToBarycentric(
                uv, info.pts2D[0], info.pts2D[1], info.pts2D[2]);
            cellFound = BarycentricPointIsInTriangle(baryCoord);
        }
    }

    // If no cell found, use a kd-Tree to find one
    if (!cellFound) {
        // Find the nearest cells
        ITKPointsLocator::NeighborsIdentifierType neighborhood;
        kdTree_->FindPointsWithinRadius(uv.val, kdMaxDist_, neighborhood);

        // Iterate through the nearest cells
        for (const auto& cell : neighborhood) {
            // Skip the cell hint we've already checked
            if (useHint and cell == cellHint) {
                continue;
            }

            // Check if this pixel is in this cell
            info = cellInformation_[cell];
            baryCoord = CartesianToBarycentric(
                uv, info.pts2D[0], info.pts2D[1], info.pts2D[2]);
            cellFound = BarycentricPointIsInTriangle(baryCoord);

            // Break if we found a matching cell
            if (cellFound) {
                useHint = true;
                cellHint = cell;
                break;
            }
        }
    }

    // If still no cell found, move to the next pixel
    if (!cellFound) {
        return;
    }

    // Find the xyz coordinate of the original point
    cv::Vec3d xyz = BarycentricToCartesian(
        baryCoord, info.pts3D[0], info.pts3D[1], info.pts3D[2]);

    // Get this pixel's normal
    cv::Vec3d xyzNorm;
    switch (shading_) {
        case Shading::Flat:
            xyzNorm = info.normals.at(0);
            break;
        case Shading::Smooth:
            xyzNorm = GouraudNormal(
                baryCoord, info.normals.at(0), info.normals.at(1),
                info.normals.at(2));
            break;
    }

    // Assign the cell index to the cell map
    cellMap_.at<int32_t>(y, x) = cellHint;

    // Assign the intensity value at the UV position
    mask_.at<uint8_t>(y, x) = MASK_TRUE;

    // Assign 3D position to the lookup map
    ppm_(y, x) =
        cv::Vec6d(xyz(0), xyz(1), xyz(2), xyzNorm(0), xyzNorm(1), xyzNorm(2));
}

// Convert from Barycentric coordinates to a smoothly interpolated normal
cv::Vec3d PPMGenerator::GouraudNormal(
    const cv::Vec3d& nUVW,
    const cv::Vec3d& nA,
    const cv::Vec3d& nB,
    const cv::Vec3d& nC)
{
    return cv::normalize(
        (1 - nUVW[0] - nUVW[1]) * nA + nUVW[1] * nB + nUVW[2] * nC);
}
