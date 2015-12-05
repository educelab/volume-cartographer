#include <algorithm>
#include <functional>
#include <numeric>
#include <chrono>
#include <cassert>
#include <tuple>

#include "chain.h"
#include "fittedcurve.h"
#include "common.h"
#include "normalizedintensitymap.h"


using namespace volcart::segmentation;

// Main constructor
Chain::Chain(VolumePkg& volpkg, int32_t zIndex) :
    volpkg_(volpkg), particleCount_(0), zIndex_(zIndex)
{
    auto segmentationPath = volpkg_.openCloud();
	decltype(curve_)::PointVectorType curvePoints;
    curvePoints.reserve(segmentationPath->size());
    for (auto path : *segmentationPath) {
        particles_.emplace_back(path.x, path.y, path.z);
        particleCount_++;
        curvePoints.emplace_back(path.x, path.y);
    }
    curve_.fitPoints(curvePoints);
}

// Constructor from explicit points
Chain::Chain(VolumePkg& volpkg, VoxelVec& pos, int32_t zIndex) :
    volpkg_(volpkg), particleCount_(0), zIndex_(zIndex)
{
    decltype(curve_)::PointVectorType curvePoints(pos.size());
    for (auto& p : pos) {
        particles_.emplace_back(p);
        particleCount_++;
        curvePoints.emplace_back(p(VC_INDEX_X), p(VC_INDEX_Y));
    }
    curve_.fitPoints(curvePoints);
}

void Chain::setNewPositions(const VoxelVec& newPositions)
{
    assert(particleCount_ == newPositions.size() &&
            "New chain positions length != particleCount_");
	std::copy(newPositions.begin(), newPositions.end(), particles_.begin());
}

// Steps all particles (with no constraints)
std::vector<VoxelVec>
Chain::stepAll(const int32_t stepNumLayers, const int32_t keepNumMaxima) const
{
    std::vector<VoxelVec> ps(particleCount_);
    for (size_t i = 0; i < particleCount_; ++i) {
        ps.at(i) = step(i, stepNumLayers, keepNumMaxima);
    }
    return ps;
}

const Voxel Chain::calculateNormal(const size_t index) const
{
    // Get average z voxel value (makes generating the reslice a little more accurate)
    double zMean = std::accumulate(particles_.begin(), particles_.end(), 0,
            [](double sum, Particle p) { return sum + p(VC_INDEX_Z); }) / particleCount_;

    // For boundary conditions, do a simple linear interpolation of the first/last 2
    // points and set the appropriate variable based on that difference in x direction.
    // y direction is handled by interpolation.
    double before, after;
    if (index == 0) {
        auto xdiff = particles_[1](VC_INDEX_X) - particles_[0](VC_INDEX_X);
        before = particles_[0](VC_INDEX_X) - xdiff;
    } else {
        before = particles_[index-1](VC_INDEX_X);
    }
    if (index == particleCount_-1) {
        auto xdiff = particles_[particleCount_-1](VC_INDEX_X) -
			         particles_[particleCount_-2](VC_INDEX_X);
        after = particles_[particleCount_-1](VC_INDEX_X) + xdiff;
    } else{
        after = particles_[index+1](VC_INDEX_X);
    }

    auto tanVec = Voxel(
			after - before, curve_.at(after) - curve_.at(before), zMean);
    return tanVec.cross(VC_DIRECTION_K);
}

// Step an individual particle
VoxelVec
Chain::step(const int32_t index, const int32_t stepNumLayers,
        const int32_t keepNumMaxima) const
{
    auto currentParticle = particles_[index];

    // Get reslice in k-direction at this point.
    const auto normal = calculateNormal(index);
    auto reslice = volpkg_.reslice(currentParticle, normal, VC_DIRECTION_K);
    auto mat = reslice.sliceData();

    // Get normalized intensity map and find the maxima
    const auto center = cv::Point(mat.cols / 2, mat.rows / 2);
    const auto map = NormalizedIntensityMap(mat.row(center.y + stepNumLayers));
    if (index == 33) {
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
    VoxelVec voxelMaxima(maxima.size());
    std::transform(maxima.begin(), maxima.end(), std::back_inserter(voxelMaxima),
        [reslice, center, stepNumLayers](IndexDistPair p) {
            return reslice.sliceCoordToVoxelCoord(
                    cv::Point(std::get<0>(p), center.y + stepNumLayers));
        });

    // XXX Go straight down if no maxima to choose from?
    if (voxelMaxima.empty()) {
        // Need to wrap straight down point in a vector.
        cv::Point newPoint(center.x, center.y + stepNumLayers);
        Voxel voxelPoint = reslice.sliceCoordToVoxelCoord(newPoint);
        return VoxelVec({ voxelPoint });
    } else {
        return voxelMaxima;
    }
}

void Chain::draw() const {
    auto pkgSlice = volpkg_.getSliceData(zIndex_).clone();
    pkgSlice /= 255.0;
    pkgSlice.convertTo(pkgSlice, CV_8UC3);
    cvtColor(pkgSlice, pkgSlice, CV_GRAY2BGR);

    // draw circles on the pkgSlice window for each point
    for (size_t i = 0; i < particleCount_; ++i) {
        auto x = particles_.at(i)(VC_INDEX_X);
        auto y = particles_.at(i)(VC_INDEX_Y);
        cv::Point real(x, y);
        cv::Point interpolated(x, curve_.at(x));
        circle(pkgSlice, real, 2, BGR_GREEN, -1);
        circle(pkgSlice, interpolated, 2, BGR_BLUE, -1);
    }

    namedWindow("Volpkg Slice", cv::WINDOW_NORMAL);
    imshow("Volpkg Slice", pkgSlice);
}
