//
// Created by Seth on 11/3/16.
//

#include <exception>

#include "core/util/FloatComparison.h"
#include "texturing/PPMGenerator.h"

using namespace volcart;
using namespace texturing;

constexpr static size_t KD_DEFAULT_SEARCH_SIZE = 100;

// Parameters
void PPMGenerator::setDimensions(size_t h, size_t w)
{
    _height = h;
    _width = w;
}

// Compute
void PPMGenerator::compute()
{
    if (_inputMesh.IsNull() || _inputMesh->GetNumberOfPoints() == 0 ||
        _inputMesh->GetNumberOfCells() == 0 || _uvMap.empty() || _width == 0 ||
        _height == 0) {
        auto msg = "Invalid input parameters";
        throw std::invalid_argument(msg);
    }

    // Make sure the storage vectors are clean
    _centroidMesh = ITKMesh::New();
    _cellInformation.clear();

    _generateCentroidMesh();
    _generatePPM();
}

// Generate the centroid mesh and other temporary data structures
void PPMGenerator::_generateCentroidMesh()
{
    ITKPoint centroid;
    CellInfo info;
    for (auto cell = _inputMesh->GetCells()->Begin();
         cell != _inputMesh->GetCells()->End(); ++cell) {
        info.reset();

        cv::Vec3d _2D, _3D;
        for (auto point = cell->Value()->PointIdsBegin();
             point != cell->Value()->PointIdsEnd(); ++point) {
            auto pointID = *point;

            _2D[0] = _uvMap.get(pointID)[0];
            _2D[1] = _uvMap.get(pointID)[1];
            _2D[2] = 0.0;
            info.Pts2D.push_back(_2D);

            _3D[0] = _inputMesh->GetPoint(pointID)[0];
            _3D[1] = _inputMesh->GetPoint(pointID)[1];
            _3D[2] = _inputMesh->GetPoint(pointID)[2];
            info.Pts3D.push_back(_3D);
        }

        // Calculate the cell centroid
        centroid = ((info.Pts2D[0] + info.Pts2D[1] + info.Pts2D[2]) / 3).val;

        // Generate the surface normal for this cell
        auto v1v0 = info.Pts3D[1] - info.Pts3D[0];
        auto v2v0 = info.Pts3D[2] - info.Pts3D[0];
        info.Normal = cv::normalize(v1v0.cross(v2v0));

        _cellInformation.push_back(info);
        _centroidMesh->SetPoint(cell.Index(), centroid);
    }
}

void PPMGenerator::_generatePPM()
{
    // Setup the output
    _ppm = PerPixelMap(_height, _width);
    cv::Mat mask = cv::Mat::zeros(_height, _width, CV_8UC1);
    _progress = 0.0;

    // Setup the search tree
    auto kdTree = ITKPointsLocator::New();
    kdTree->SetPoints(_centroidMesh->GetPoints());
    kdTree->Initialize();
    auto kdSearchSize =
        (_centroidMesh->GetNumberOfPoints() < KD_DEFAULT_SEARCH_SIZE)
            ? _centroidMesh->GetNumberOfPoints()
            : KD_DEFAULT_SEARCH_SIZE;
    ITKPointsLocator::NeighborsIdentifierType neighborhood;

    // Iterate over all of the pixels
    size_t pixelsNotInCell = 0;
    for (size_t y = 0; y < _height; ++y) {
        for (size_t x = 0; x < _width; ++x) {
            _progress = (x + 1.0 + (_width * y)) * 100.0 / (_width * _height);
            std::cerr << "volcart::texturing::PPMGenerator: "
                      << std::to_string(_progress) << "%\r" << std::flush;

            // This pixel's uv coordinate
            cv::Vec3d uv{0, 0, 0};
            uv[0] = x / (_width - 1.0);
            uv[1] = y / (_height - 1.0);

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
            for (auto c_id = neighborhood.begin(); c_id != neighborhood.end();
                 ++c_id) {
                info = _cellInformation[*c_id];

                // Calculate the 3D correspondence for this pixel
                baryCoord = _BarycentricCoord(
                    uv, info.Pts2D[0], info.Pts2D[1], info.Pts2D[2]);
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
            cv::Vec3d xyz = _CartesianCoord(
                baryCoord, info.Pts3D[0], info.Pts3D[1], info.Pts3D[2]);

            // Use the cell normal as the normal for this point
            cv::Vec3d xyz_norm = info.Normal;

            // Assign the intensity value at the UV position
            mask.at<uint8_t>(y, x) = 255;

            // Assign 3D position to the lookup map
            _ppm(y, x) = cv::Vec6d(
                xyz(0), xyz(1), xyz(2), xyz_norm(0), xyz_norm(1), xyz_norm(2));
        }
    }

    // Finish setting up the output
    _ppm.setUVMap(_uvMap);
    _ppm.setMask(mask);

    std::cerr << std::endl;
    std::cerr << "volcart::texturing::PPMGenerator:: Pixels not in cell: "
              << pixelsNotInCell << std::endl;
}

// Find barycentric coordinates of point in triangle
// From Christer Ericson's Real-Time Collision Detection
// Code from:
// http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
cv::Vec3d PPMGenerator::_BarycentricCoord(
    const cv::Vec3d nXYZ,
    const cv::Vec3d nA,
    const cv::Vec3d nB,
    const cv::Vec3d nC)
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
cv::Vec3d PPMGenerator::_CartesianCoord(
    const cv::Vec3d nUVW,
    const cv::Vec3d nA,
    const cv::Vec3d nB,
    const cv::Vec3d nC)
{
    return nUVW[0] * nA + nUVW[1] * nB + nUVW[2] * nC;
}
