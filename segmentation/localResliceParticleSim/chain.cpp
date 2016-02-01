#include <algorithm>
#include <functional>
#include <numeric>
#include <chrono>
#include <cassert>
#include <tuple>
#include "chain.h"
#include "normalizedintensitymap.h"

#define DRAW_INDEX 17

using namespace volcart::segmentation;

void Chain::setNewPositions(const VoxelVec& newPositions)
{
    assert(particleCount_ == newPositions.size() &&
           "New chain positions length != particleCount_");
    std::copy(newPositions.begin(), newPositions.end(), particles_.begin());
}

// Steps all particles (with no constraints)
std::vector<std::deque<Voxel>> Chain::stepAll(const int32_t stepNumLayers,
                                              const int32_t keepNumMaxima) const
{
    std::vector<std::deque<Voxel>> ps;
    ps.reserve(particleCount_);
    for (int32_t i = 0; i < particleCount_; ++i) {
        ps.push_back(step(i, stepNumLayers, keepNumMaxima));
    }
    return ps;
}

// Use structure tensor estimate for normal
cv::Vec3d Chain::calculateNormal(const size_t index) const
{
    const auto tanPixel = curve_.derivAt(index);
    const auto tanVec = Voxel(tanPixel(0), tanPixel(1), zIndex_);
    return tanVec.cross(VC_DIRECTION_K);
}

// Step an individual particle
// Returns: a std::deque<Voxel> which is used in the underlying algorithm to
// efficiently pop Voxel positions from the front
std::deque<Voxel> Chain::step(const int32_t index, const int32_t stepNumLayers,
                              const int32_t keepNumMaxima) const
{
    const Voxel currentParticle = particles_[index];

    // Get reslice in k-direction at this point.
    const cv::Vec3d normal = calculateNormal(index);
    auto reslice =
        volpkg_.volume().reslice(currentParticle, normal, VC_DIRECTION_K);
    const cv::Mat mat = reslice.sliceData();

    // Get normalized intensity map and find the maxima
    const cv::Point center(mat.cols / 2, mat.rows / 2);
    const NormalizedIntensityMap map(mat.row(center.y + stepNumLayers));
    if (index == DRAW_INDEX) {
        reslice.draw();
        map.draw();
    }
    auto maxima = map.findMaxima();

    // Sort maxima by whichever is closest to current index of center (using
    // standard euclidean 1D distance)
    std::sort(maxima.begin(), maxima.end(),
              [center](IndexDistPair lhs, IndexDistPair rhs) {
                  const auto x = center.x;
                  const auto ldist = std::abs(int32_t(lhs.first - x));
                  const auto rdist = std::abs(int32_t(rhs.first - x));
                  return ldist < rdist;
              });

    // Take only top N maxima (currently 3)
    maxima.resize(keepNumMaxima, IndexDistPair(0, 0));
    for (int32_t i = keepNumMaxima - 1; i >= 0; --i) {
        if (std::get<0>(maxima[i]) == 0 && std::get<1>(maxima[i]) == 0) {
            maxima.pop_back();
        }
    }

    // Convert from pixel space to voxel space
    std::deque<Voxel> voxelMaxima;
    for (const IndexDistPair p : maxima) {
        voxelMaxima.emplace_back(reslice.sliceCoordToVoxelCoord(
            {p.first, center.y + stepNumLayers}));
    }

    // XXX Go straight down if no maxima to choose from?
    if (voxelMaxima.empty()) {
        // Need to wrap straight down point in a vector.
        Voxel straightDownPoint = reslice.sliceCoordToVoxelCoord(
            {center.x, center.y + stepNumLayers});
        return std::deque<Voxel>({straightDownPoint});
    } else {
        return voxelMaxima;
    }
}

void Chain::draw() const
{
    auto pkgSlice = volpkg_.volume().getSliceData(zIndex_).clone();
    pkgSlice /= 255.0;
    pkgSlice.convertTo(pkgSlice, CV_8UC3);
    cvtColor(pkgSlice, pkgSlice, CV_GRAY2BGR);

    // draw circles on the pkgSlice window for each point
    for (size_t i = 0; i < particleCount_; ++i) {
        auto x = particles_[i](0);
        auto y = particles_[i](1);
        cv::Point real(x, y);
        if (i == DRAW_INDEX) {
            cv::circle(pkgSlice, real, 1, BGR_RED, -1);
        } else {
            cv::circle(pkgSlice, real, 1, BGR_GREEN, -1);
        }
    }

    cv::namedWindow("Volpkg Slice", cv::WINDOW_NORMAL);
    cv::imshow("Volpkg Slice", pkgSlice);
}
