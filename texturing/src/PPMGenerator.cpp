//
// Created by Seth on 11/3/16.
//

#include <stdexcept>

#include "vc/core/util/FloatComparison.hpp"
#include "vc/meshing/CalculateNormals.hpp"
#include "vc/meshing/DeepCopy.hpp"
#include "vc/texturing/PPMGenerator.hpp"

using namespace volcart;
using namespace texturing;

namespace vcm = volcart::meshing;

constexpr static size_t KD_DEFAULT_SEARCH_SIZE = 100;

// Parameters
void PPMGenerator::setDimensions(size_t h, size_t w)
{
    height_ = h;
    width_ = w;
}

// Compute
PerPixelMap& PPMGenerator::compute()
{
    if (inputMesh_.IsNull() || inputMesh_->GetNumberOfPoints() == 0 ||
        inputMesh_->GetNumberOfCells() == 0 || uvMap_.empty() || width_ == 0 ||
        height_ == 0) {
        auto msg = "Invalid input parameters";
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

        cv::Vec3d twoD, threeD, normal;
        for (auto point = cell->Value()->PointIdsBegin();
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

        // Calculate the cell centroid
        centroid = ((info.pts2D[0] + info.pts2D[1] + info.pts2D[2]) / 3).val;

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
    cv::Mat mask = cv::Mat::zeros(height_, width_, CV_8UC1);
    progress_ = 0.0;

    // Setup the search tree
    auto kdTree = ITKPointsLocator::New();
    kdTree->SetPoints(centroidMesh_->GetPoints());
    kdTree->Initialize();
    auto kdSearchSize =
        (centroidMesh_->GetNumberOfPoints() < KD_DEFAULT_SEARCH_SIZE)
            ? centroidMesh_->GetNumberOfPoints()
            : KD_DEFAULT_SEARCH_SIZE;
    ITKPointsLocator::NeighborsIdentifierType neighborhood;

    // Iterate over all of the pixels
    size_t pixelsNotInCell = 0;
    for (size_t y = 0; y < height_; ++y) {
        for (size_t x = 0; x < width_; ++x) {
            // This pixel's uv coordinate
            cv::Vec3d uv{0, 0, 0};
            uv[0] = x / (width_ - 1.0);
            uv[1] = y / (height_ - 1.0);

            // Empty our averaging variables
            neighborhood.clear();

            // find k nearest neighbors for current point
            ITKPoint searchPoint;
            searchPoint[0] = uv[0];
            searchPoint[1] = uv[1];
            searchPoint[2] = 0.0;
            kdTree->FindClosestNPoints(searchPoint, kdSearchSize, neighborhood);

            // Find which triangle this pixel lies inside of
            // Is the current pixel in this cell?
            auto in2D = false;
            CellInfo info;
            cv::Vec3d baryCoord{0, 0, 0};
            for (auto cellId : neighborhood) {
                info = cellInformation_[cellId];

                // Calculate the 3D correspondence for this pixel
                baryCoord = barycentric_coord_(
                    uv, info.pts2D[0], info.pts2D[1], info.pts2D[2]);
                in2D =
                    ((baryCoord[0] > 0.0 || AlmostEqual(baryCoord[0], 0.0)) &&
                     (baryCoord[1] > 0.0 || AlmostEqual(baryCoord[1], 0.0)) &&
                     (baryCoord[2] > 0.0 || AlmostEqual(baryCoord[2], 0.0)) &&
                     (baryCoord[0] + baryCoord[1] < 1.0 ||
                      AlmostEqual(baryCoord[0] + baryCoord[1], 1.0)));

                if (in2D) {
                    break;
                }
            }

            // Set this pixel to black if not part of a cell
            if (!in2D) {
                ++pixelsNotInCell;
                continue;
            }

            // Find the xyz coordinate of the original point
            cv::Vec3d xyz = cartesian_coord_(
                baryCoord, info.pts3D[0], info.pts3D[1], info.pts3D[2]);

            // Get the point's normal
            cv::Vec3d xyzNorm;
            switch (shading_) {
                case Shading::Flat:
                    xyzNorm = info.normals.at(0);
                    break;
                case Shading::Smooth:
                    xyzNorm = gouraud_normal_(
                        baryCoord, info.normals.at(0), info.normals.at(1),
                        info.normals.at(2));
                    break;
            }

            // Assign the intensity value at the UV position
            mask.at<uint8_t>(y, x) = 255;

            // Assign 3D position to the lookup map
            ppm_(y, x) = cv::Vec6d(
                xyz(0), xyz(1), xyz(2), xyzNorm(0), xyzNorm(1), xyzNorm(2));
        }
    }

    // Finish setting up the output
    ppm_.setUVMap(uvMap_);
    ppm_.setMask(mask);
}

// Find barycentric coordinates of point in triangle
// From Christer Ericson's Real-Time Collision Detection
// Code from:
// http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
cv::Vec3d PPMGenerator::barycentric_coord_(
    const cv::Vec3d& nXYZ,
    const cv::Vec3d& nA,
    const cv::Vec3d& nB,
    const cv::Vec3d& nC)
{
    auto v0 = nB - nA;
    auto v1 = nC - nA;
    auto v2 = nXYZ - nA;

    auto dot00 = v0.dot(v0);
    auto dot01 = v0.dot(v1);
    auto dot11 = v1.dot(v1);
    auto dot20 = v2.dot(v0);
    auto dot21 = v2.dot(v1);
    auto invDenom = 1 / (dot00 * dot11 - dot01 * dot01);

    cv::Vec3d output;
    output[1] = (dot11 * dot20 - dot01 * dot21) * invDenom;
    output[2] = (dot00 * dot21 - dot01 * dot20) * invDenom;
    output[0] = 1.0 - output[1] - output[2];

    return output;
}

// Find Cartesian coordinates of point in triangle given barycentric coordinate
cv::Vec3d PPMGenerator::cartesian_coord_(
    const cv::Vec3d& nUVW,
    const cv::Vec3d& nA,
    const cv::Vec3d& nB,
    const cv::Vec3d& nC)
{
    return nUVW[0] * nA + nUVW[1] * nB + nUVW[2] * nC;
}

// Convert from Barycentric coordinates to a smoothly interpolated normal
cv::Vec3d PPMGenerator::gouraud_normal_(
    const cv::Vec3d& nUVW,
    const cv::Vec3d& nA,
    const cv::Vec3d& nB,
    const cv::Vec3d& nC)
{
    return cv::normalize(
        (1 - nUVW[0] - nUVW[1]) * nA + nUVW[1] * nB + nUVW[2] * nC);
}