#include <algorithm>
#include <functional>
#include <numeric>
#include <chrono>
#include <cassert>
#include <tuple>

#include "chain.h"
#include "common.h"
#include "NormalizedIntensityMap.h"


using namespace volcart::segmentation;

// Main constructor
Chain::Chain(VolumePkg& volpkg, int32_t zIndex) :
    volpkg_(volpkg), particleCount_(0), zIndex_(zIndex)
{
    auto segmentationPath = volpkg_.openCloud();
    auto curvePoints = std::vector<std::tuple<double, double>>();
    curvePoints.reserve(segmentationPath->size());
    for (auto path : *segmentationPath) {
        particles_.emplace_back(path.x, path.y, path.z);
        particleCount_++;
        curvePoints.emplace_back(path.x, path.y);
    }
    curve_.fitPoints(curvePoints);
}

void Chain::setNewPositions(std::vector<cv::Vec3d> newPositions)
{
    assert(particleCount_ == newPositions.size() &&
            "New chain positions length != particleCount_");
    for (size_t i = 0; i < particleCount_; ++i) {
        particles_[i] = newPositions.at(i);
    }
}

// Steps all particles (with no constraints)
Chain::DirPosPairVec Chain::stepAll(const int32_t stepNumLayers) const
{
    auto positions = std::vector<cv::Vec3d>(particleCount_);
    auto directions = std::vector<Direction>(particleCount_);
    for (size_t i = 0; i < particleCount_; ++i) {
        std::tie(directions[i], positions[i]) = step(i, stepNumLayers);
    }
    return std::make_tuple(directions, positions);
}

const cv::Vec3d Chain::calculateNormal(const size_t index) const
{
    // Get average z voxel value (makes generating the reslice a little more accurate)
    double zMean = std::accumulate(particles_.begin(), particles_.end(), 0,
            [](double sum, Particle p) { return sum + p.z(); }) / particleCount_;

    // For boundary conditions, do a simple linear interpolation of the first/last 2
    // points and set the appropriate variable based on that difference in x direction.
    // y direction is handled by interpolation.
    double before, after;
    if (index == 0) {
        auto xdiff = particles_[1].x() - particles_[0].x();
        before = particles_[0].x() - xdiff;
    } else {
        before = particles_[index-1].x();
    }
    if (index == particleCount_-1) {
        auto xdiff = particles_[particleCount_-1].x() - particles_[particleCount_-2].x();
        after = particles_[particleCount_-1].x() + xdiff;
    } else{
        after = particles_[index+1].x();
    }

    auto tanVec = cv::Vec3d(after - before, curve_.at(after) - curve_.at(before), zMean);
    return tanVec.cross(VC_DIRECTION_K);
}

// Step an individual particle with optional direction and drift constraints
Chain::DirPosPair Chain::step(const int32_t index, const int32_t stepNumLayers,
        const Direction dirConstraint, double maxDrift) const
{
    auto currentParticle = particles_[index];
    if (!currentParticle.isMoving()) {
        // XXX What to return here if it's not moving? Assume they all move continuously for now
    }

    // Get reslice in k-direction at this point.
    const auto normal = calculateNormal(index);
    auto reslice = volpkg_.reslice(currentParticle.position(), normal, VC_DIRECTION_K);
    auto mat = reslice.sliceData();

    // Calculate the next position for this particle
    const auto center = cv::Point(mat.cols / 2, mat.rows / 2);
    const auto map = NormalizedIntensityMap(mat.row(center.y + stepNumLayers));
    if (index == 33) {
        reslice.draw();
        map.draw();
    }
    auto maxima = map.findMaxima(index);

    // Sort maxima by whichever is closest to current index of center (using
    // standard euclidean 1D distance)
    std::sort(maxima.begin(), maxima.end(),
        [center](IndexDistPair lhs, IndexDistPair rhs) {
            const auto x = center.x;
            const auto ldist = std::abs(int32_t(lhs.first - x));
            const auto rdist = std::abs(int32_t(rhs.first - x));
            return ldist < rdist;
        });

    // Convert from pixel space to voxel space and enforce constraints
    auto voxelMaxima = std::vector<DirPosPair>();
    voxelMaxima.reserve(maxima.size());
    std::transform(maxima.begin(), maxima.end(), std::back_inserter(voxelMaxima),
        [reslice, center, stepNumLayers](IndexDistPair p) {
            Direction d;
            if (int32_t(std::get<0>(p) - center.y) > 0) {
                d = Direction::kLeft;
            } else if (int32_t(std::get<0>(p) - center.y) < 0) {
                d = Direction::kRight;
            } else {
                d = Direction::kNone;
            }
            const auto voxel = reslice.sliceCoordToVoxelCoord(
                    cv::Point(std::get<0>(p), center.y + stepNumLayers));
            return std::make_tuple(d, voxel);
        });

    // Remove any pairs that don't satisfy the direction constraint
    // (doesn't remove Direction::kNone by default)
    // XXX assumes that dirConstraint is either Direction::kLeft or Direction::kRight
    if (dirConstraint != Direction::kDefault) {
        std::remove_if(voxelMaxima.begin(), voxelMaxima.end(),
            [center, dirConstraint](DirPosPair p) {
                return dirConstraint != std::get<0>(p);
            });
    }

    // Remove any pairs that don't satisfy the position constraint
    if (maxDrift != kDefaultMaxDrift) {
        std::remove_if(voxelMaxima.begin(), voxelMaxima.end(),
            [maxDrift, currentParticle](DirPosPair p) {
                return cv::norm(std::get<1>(p) - currentParticle.position()) > maxDrift;
            });
    }

    // XXX Go straight down if no maxima to choose from?
    if (voxelMaxima.empty()) {
        auto newPoint = cv::Point(center.x, center.y + stepNumLayers);
        return std::make_tuple(
                Direction::kNone,
                reslice.sliceCoordToVoxelCoord(newPoint));
    } else {
        return voxelMaxima.at(0);
    }
}

void Chain::draw() const {
    auto pkgSlice = volpkg_.getSliceData(zIndex_).clone();
    pkgSlice /= 255.0;
    pkgSlice.convertTo(pkgSlice, CV_8UC3);
    cvtColor(pkgSlice, pkgSlice, CV_GRAY2BGR);

    // draw circles on the pkgSlice window for each point
    for (size_t i = 0; i < particleCount_; ++i) {
        auto x = particles_.at(i).x();
        auto y = particles_.at(i).y();
        cv::Point real(x, y);
        cv::Point interpolated(x, curve_.at(x));
        circle(pkgSlice, real, 2, BGR_GREEN, -1);
        circle(pkgSlice, interpolated, 2, BGR_BLUE, -1);
    }

    namedWindow("Volpkg Slice", cv::WINDOW_NORMAL);
    imshow("Volpkg Slice", pkgSlice);
}
