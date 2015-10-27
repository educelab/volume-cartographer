#include <algorithm>
#include <functional>
#include <chrono>
#include <cassert>

#include "chain.h"
#include "NormalizedIntensityMap.h"


using namespace volcart::segmentation;

// Main constructor
Chain::Chain(VolumePkg& volpkg, int32_t zIndex) :
    volpkg_(volpkg), particleCount_(0), zIndex_(zIndex)
{
    auto segmentationPath = volpkg_.openCloud();
    for (auto path : *segmentationPath) {
        particles_.emplace_back(path.x, path.y, path.z);
        particleCount_++;
    }
    std::cout << "particleCount_ = " << particleCount_ << std::endl;
}

Particle Chain::at(const int32_t idx) const
{
    return particles_.at(idx);
}

void Chain::setNewPositions(std::vector<cv::Vec3f> newPositions)
{
    assert(particleCount_ == newPositions.size() && "New chain positions length != particleCount_");
    for (size_t i = 0; i < particleCount_; ++i) {
        particles_[i] = newPositions.at(i);
    }
}

// Steps all particles (with no constraints)
Chain::DirPosVecPair Chain::stepAll() const
{
    auto positions = std::vector<cv::Vec3f>();
    positions.reserve(particleCount_);
    auto directions = std::vector<Direction>();
    directions.reserve(particleCount_);
    for (size_t i = 0; i < particleCount_; ++i) {
        std::tie(directions[i], positions[i]) = step(i);
    }
    return std::make_tuple(directions, positions);
}

const cv::Vec3f Chain::calculateNormal(const int32_t index) const
{
    // Change indexing if we're at the front or back particle
    auto i = index;
    if (index == 0) {
        i = 1;
    } else if (uint32_t(index) == particleCount_ - 1) {
        i = particleCount_ - 2;
    }

    auto diff = particles_[i+1] - particles_[i-1];
    auto tanVec = cv::Vec3f(diff(VC_INDEX_X), diff(VC_INDEX_Y), zIndex_);
    return tanVec.cross(VC_DIRECTION_K);
}

// Step an individual particle with optional direction and drift constraints
Chain::DirPosPair Chain::step(const int32_t index, const Direction dirConstraint, double maxDrift) const
{
    auto currentParticle = particles_[index];
    if (!currentParticle.isMoving()) {
        // XXX What to return here if it's not moving? Assume they all move continuously for now
    }

    // Get reslice in k-direction at this point.
    const auto normal = calculateNormal(index);
    auto reslice = volpkg_.reslice(currentParticle.position(), normal, VC_DIRECTION_K);
    if (index == 29) {
        reslice.draw();
    }
    auto mat = reslice.sliceData();

    // Calculate the next position for this particle
    constexpr auto lookaheadDepth = 2;
    const auto center = cv::Point(mat.cols / 2, mat.rows / 2);
    const auto map = NormalizedIntensityMap(mat.row(center.y + lookaheadDepth));
    auto maxima = map.findMaxima();

    // Sort maxima by whichever is closest to current index of center (using standard euclidean 1D distance)
    using IndexDistPair = std::pair<int32_t, double>;
    std::sort(maxima.begin(), maxima.end(), [center](IndexDistPair lhs, IndexDistPair rhs) {
        const auto x = center.x;
        const auto ldist = std::abs(int32_t(lhs.first - x));
        const auto rdist = std::abs(int32_t(rhs.first - x));
        return ldist < rdist;
    });

    // Convert from pixel space to voxel space and enforce constraints
    auto voxelMaxima = std::vector<DirPosPair>();
    voxelMaxima.reserve(maxima.size());
    std::transform(maxima.begin(), maxima.end(), std::back_inserter(voxelMaxima),
        [reslice, center](IndexDistPair p) {
            Direction d;
            if (int32_t(std::get<0>(p) - center.y) > 0) {
                d = Direction::kLeft;
            } else if (int32_t(std::get<0>(p) - center.y) < 0) {
                d = Direction::kRight;
            } else {
                d = Direction::kNone;
            }
            const auto voxel = reslice.sliceCoordToVoxelCoord(
                    cv::Point(std::get<0>(p), center.y + lookaheadDepth));
            return std::make_tuple(d, voxel);
        });

    // Remove any pairs that don't satisfy the direction constraint
    // (doesn't remove Direction::kNone by default)
    // XXX assumes that dirConstraint is either Direction::kLeft or Direction::kRight
    if (dirConstraint != Direction::kNone) {
        std::remove_if(voxelMaxima.begin(), voxelMaxima.end(), [center, dirConstraint](DirPosPair p) {
            return (std::get<0>(p) != Direction::kNone) && (dirConstraint != std::get<0>(p));
        });
    }

    // Remove any pairs that don't satisfy the position constraint
    if (maxDrift != kDefaultMaxDrift) {
        std::remove_if(voxelMaxima.begin(), voxelMaxima.end(), [maxDrift, currentParticle](DirPosPair p) {
            return cv::norm(std::get<1>(p) - currentParticle.position()) > maxDrift;
        });
    }

    // XXX Go straight down if no maxima to choose from?
    if (voxelMaxima.empty()) {
        auto newPoint = cv::Point(center.x, center.y + lookaheadDepth);
        return std::make_tuple(Direction::kNone, reslice.sliceCoordToVoxelCoord(newPoint));
    } else {
        return voxelMaxima.at(0);
    }
}

void Chain::draw() const {
    auto pkgSlice = volpkg_.getSliceData(zIndex_);
    pkgSlice /= 255.0;
    pkgSlice.convertTo(pkgSlice, CV_8UC3);
    cvtColor(pkgSlice, pkgSlice, CV_GRAY2BGR);

    // draw circles on the pkgSlice window for each point
    for (size_t i = 0; i < particleCount_; ++i) {
        cv::Point position(particles_.at(i)(VC_INDEX_X), particles_.at(i)(VC_INDEX_Y));
        if (i == 32) {
            circle(pkgSlice, position, 2, BGR_YELLOW, -1);
        } else {
            circle(pkgSlice, position, 2, BGR_GREEN, -1);
        }
    }

    namedWindow("Volpkg Slice", cv::WINDOW_NORMAL);
    imshow("Volpkg Slice", pkgSlice);
}
