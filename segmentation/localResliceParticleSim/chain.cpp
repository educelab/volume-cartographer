#include <algorithm>
#include <functional>

#include "chain.h"
#include "NormalizedIntensityMap.h"

using namespace volcart::segmentation;

// Main constructor
Chain::Chain(VolumePkg& volpkg) : volpkg_(volpkg) {
    auto segmentationPath = volpkg_.openCloud();
    for (auto path : *segmentationPath) {
        particles_.push_back(Particle(path.x, path.y, path.z));
    }
}

Particle Chain::at(const int32_t idx) const
{
    return particles_.at(idx);
}

int32_t Chain::zIndex(void) const
{
    double meanZIdx = 0.0;
    for (auto p : particles_) {
        meanZIdx += p(VC_INDEX_Z);
    }
    return cvRound(meanZIdx);
}

void Chain::setNewPositions(std::vector<cv::Vec3f> newPositions)
{
    if (particles_.size() != newPositions.size()) {
        std::cerr << "newPositions vector is not the same size as current particles size" << std::endl;
        return;
    }
    for (auto i = 0; i < particleCount_; ++i) {
        particles_[i] = newPositions.at(i);
    }
}

// Steps all particles (with no constraints)
std::tuple<std::vector<Direction>, std::vector<cv::Vec3f>> Chain::stepAll() const
{
    auto positions = std::vector<cv::Vec3f>(particleCount_);
    auto directions = std::vector<Direction>(particleCount_);
    for (auto i = 0; i < particleCount_; ++i) {
        std::tie(directions[i], positions[i]) = step(i);
    }
    return std::make_tuple(directions, positions);
}

cv::Vec3f Chain::calculateNormal(const int32_t index) const
{
    // Create N x 3 matrix from chain
    int32_t N = int32_t(particles_.size());
    auto matChain = cv::Mat(N, 2, CV_32F);
    auto matChainZ = cv::Mat(N, 1, CV_32F);
    for (int32_t i = 0; i < N; ++i) {
        matChain.at<float>(i, VC_INDEX_X) = particles_[i].position()(VC_INDEX_X);
        matChain.at<float>(i, VC_INDEX_Y) = particles_[i].position()(VC_INDEX_Y);
        matChainZ.at<float>(i, 0) = particles_[i].position()(VC_INDEX_Z);
    }
    auto mean = cv::mean(matChainZ)(0);

    // Change indexing if we're at the front or back particle
    int32_t i = index;
    if (index == 0) {
        i = 1;
    } else if (index == N - 1) {
        i = particleCount_ - 2;
    }

    auto tangent = cv::Point2f(matChain.at<float>(i+1) - matChain.at<float>(i-1));
    auto tanVec = cv::Vec3f(tangent.x, tangent.y, mean);
    return tanVec.cross(VC_DIRECTION_K);
}

// Step an individual particle with optional direction and drift constraints
std::tuple<Direction, cv::Vec3f> Chain::step(const int32_t index, const Direction dirConstraint, double maxDrift) const
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
    auto center = cv::Point(mat.cols / 2, mat.rows / 2);

    const auto map = NormalizedIntensityMap(mat.row(center.y + lookaheadDepth));
    auto maxima = map.findMaxima();

    // Sort maxima by whichever is closest to current index of center (using standard euclidean 1D distance)
    using IndexDistPair = std::pair<int32_t, double>;
    std::sort(maxima.begin(), maxima.end(), [center](IndexDistPair lhs, IndexDistPair rhs) {
        auto x = center.x;
        auto ldist = std::abs(int32_t(lhs.first - x));
        auto rdist = std::abs(int32_t(rhs.first - x));
        return ldist < rdist;
    });

    // Convert from pixel space to voxel space to enforce constraints (if necessary)
    using DirPosPair = std::tuple<Direction, cv::Vec3f>;
    auto voxelMaxima = std::vector<DirPosPair>(maxima.size());
    std::transform(maxima.begin(), maxima.end(), std::back_inserter(voxelMaxima), [reslice, center](IndexDistPair p) {
        Direction d;
        if (int32_t(std::get<0>(p) - center.y) > 0) {
            d = Direction::kLeft;
        } else if (int32_t(std::get<0>(p) - center.y) < 0) {
            d = Direction::kRight;
        } else {
            d = Direction::kNone;
        }
        auto voxel = reslice.sliceCoordToVoxelCoord(cv::Point(std::get<0>(p), center.y + lookaheadDepth));
        return std::make_tuple(d, voxel);
    });

    // Remove any pairs that don't satisfy the direction constraint (default behavior is to not remove any)
    // XXX assumes that dirConstraint is either Direction::kLeft or Direction::kRight
    std::remove_if(voxelMaxima.begin(), voxelMaxima.end(), [center, dirConstraint](DirPosPair p) {
        return (std::get<0>(p) != Direction::kNone) && (dirConstraint != std::get<0>(p));
    });

    // Remove any pairs that don't satisfy the position constraint
    if (maxDrift != kDefaultMaxDrift) {
        std::remove_if(voxelMaxima.begin(), voxelMaxima.end(), [maxDrift, currentParticle](DirPosPair p) {
            return cv::norm(std::get<1>(p) - currentParticle.position()) > maxDrift;
        });
    }

    // XXX What do we do if there's nothing left in the voxelMaxima at this point?

    // Find next point in slice space
    return voxelMaxima.at(0);
}

void Chain::draw() const {
    auto zidx = zIndex();
    auto pkgSlice = volpkg_.getSliceData(zidx);
    pkgSlice /= 255.0;
    pkgSlice.convertTo(pkgSlice, CV_8UC3);
    cvtColor(pkgSlice, pkgSlice, CV_GRAY2BGR);

    // draw circles on the pkgSlice window for each point
    for (int32_t i = 0; i < particleCount_; ++i) {
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
