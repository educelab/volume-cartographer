#include <algorithm>
#include <functional>
#include <numeric>
#include <chrono>
#include <cassert>
#include <cmath>
#include <tuple>
#include "chain.h"
#include "normalizedintensitymap.h"

#define DRAW_INDEX 17

using namespace volcart::segmentation;

void Chain::setNewPositions(const VoxelVec& newPositions)
{
    assert(particleCount_ == static_cast<int32_t>(newPositions.size()) &&
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

// Use spline for curve tangent estimate
cv::Vec3d Chain::calculateNormal(const size_t index) const
{
    const Voxel currentVoxel = particles_[index];
    const auto eigenPairs = volpkg_.volume().eigenPairsAtIndex(
        currentVoxel(0), currentVoxel(1), currentVoxel(2), 3);
    const double exp0 = std::log10(eigenPairs[0].first);
    const double exp1 = std::log10(eigenPairs[1].first);
    std::cout << std::abs(exp0 - exp1) << std::endl;
    if (std::abs(exp0 - exp1) > 2.0) {
        return eigenPairs[0].second;
    }
    const auto tanPixel = curve_.derivAt(index, 3);
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
    // standard euclidean 1D distance) weighted by the intensity of each side
    std::sort(maxima.begin(), maxima.end(),
              [center, this](IndexIntensityPair lhs, IndexIntensityPair rhs) {
                  /*
                  const double ldist =
                      2 * std::abs(lhs.first - center.x) / particleCount_;
                  const double rdist =
                      2 * std::abs(rhs.first - center.x) / particleCount_;
                  const int32_t distWeight = 75;
                  return (distWeight * ldist + (100 - distWeight) * -lhs.second)
                  <
                         (distWeight * rdist + (100 - distWeight) *
                  -rhs.second);
                         */
                  const int32_t ldist =
                      static_cast<int32_t>(std::abs(lhs.first - center.x));
                  const int32_t rdist =
                      static_cast<int32_t>(std::abs(rhs.first - center.x));
                  return ldist < rdist;
              });

    // Take only top N maxima
    maxima.resize(keepNumMaxima, IndexIntensityPair(0, 0));
    for (int32_t i = keepNumMaxima - 1; i >= 0; --i) {
        if (std::get<0>(maxima[i]) == 0 && std::get<1>(maxima[i]) == 0) {
            maxima.pop_back();
        }
    }

    // Convert from pixel space to voxel space
    std::deque<Voxel> voxelMaxima;
    for (const auto p : maxima) {
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
    for (int32_t i = 0; i < particleCount_; ++i) {
        auto x = particles_[i](0);
        auto y = particles_[i](1);
        cv::Point real(x, y);
        if (i == DRAW_INDEX) {
            cv::circle(pkgSlice, real, 1, BGR_RED, -1);
        } else {
            cv::circle(pkgSlice, real, 1, BGR_GREEN, -1);
        }
    }

    // Superimpose interpolated curve on window
    double sum = 0;
    for (int32_t i = 0; i < particleCount_ && sum <= 1;
         ++i, sum += 1.0 / particleCount_) {
        cv::Point p(curve_.eval(sum));
        cv::circle(pkgSlice, p, 1, BGR_BLUE, -1);
    }

    cv::namedWindow("Volpkg Slice", cv::WINDOW_NORMAL);
    cv::imshow("Volpkg Slice", pkgSlice);
}
