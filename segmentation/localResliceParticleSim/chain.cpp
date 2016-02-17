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
	decltype(curve_)::ScalarVector xvals, yvals;
    xvals.reserve(segmentationPath->size());
    yvals.reserve(segmentationPath->size());
    for (auto path : *segmentationPath) {
        xvals.push_back(path.x);
        yvals.push_back(path.y);
		particleCount_++;
    }
    curve_ = FittedCurve<double>(xvals, yvals, particleCount_ / 2);
	auto particles2d = curve_.resampledPoints();
	for (const auto& p : particles2d) {
		particles_.emplace_back(p(0), p(1), zIndex_);
	}
}

// Constructor from explicit points
Chain::Chain(VolumePkg& volpkg, const VoxelVec& pos, int32_t zIndex) :
    volpkg_(volpkg), particleCount_(0), zIndex_(zIndex)
{
	decltype(curve_)::ScalarVector xvals, yvals;
    xvals.reserve(pos.size());
    yvals.reserve(pos.size());
    particles_.reserve(pos.size());
    for (const auto& p : pos) {
        xvals.push_back(p(VC_INDEX_X));
        yvals.push_back(p(VC_INDEX_Y));
		particleCount_++;
    }
    curve_ = FittedCurve<double>(xvals, yvals, particleCount_);
	auto particles2d = curve_.resampledPoints();
	for (const auto& p : particles2d) {
		particles_.emplace_back(p(0), p(1), zIndex_);
	}
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
    std::vector<VoxelVec> ps;
    ps.reserve(particleCount_);
    for (size_t i = 0; i < particleCount_; ++i) {
        ps.push_back(step(i, stepNumLayers, keepNumMaxima));
    }
    return ps;
}

Voxel Chain::calculateNormal(const size_t index) const
{
	const auto tanPixel = curve_.derivAt(index);
	const auto tanVec = Voxel(tanPixel(VC_INDEX_X), tanPixel(VC_INDEX_Y), zIndex_);
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
    auto reslice = volpkg_.volume().reslice(currentParticle, normal, VC_DIRECTION_K);
    const auto mat = reslice.sliceData();

    // Get normalized intensity map and find the maxima
    const auto center = cv::Point(mat.cols / 2, mat.rows / 2);
    const auto map = NormalizedIntensityMap(mat.row(center.y + stepNumLayers));
    if (index == 47) {
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
    VoxelVec voxelMaxima;
    voxelMaxima.reserve(maxima.size());
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
    auto pkgSlice = volpkg_.volume().getSliceData(zIndex_).clone();
    pkgSlice /= 255.0;
    pkgSlice.convertTo(pkgSlice, CV_8UC3);
    cvtColor(pkgSlice, pkgSlice, CV_GRAY2BGR);

    // draw circles on the pkgSlice window for each point
    for (size_t i = 0; i < particleCount_; ++i) {
        auto x = particles_.at(i)(VC_INDEX_X);
        auto y = particles_.at(i)(VC_INDEX_Y);
        cv::Point real(x, y);
        circle(pkgSlice, real, 1, BGR_GREEN, -1);
    }

    namedWindow("Volpkg Slice", cv::WINDOW_NORMAL);
    imshow("Volpkg Slice", pkgSlice);
}
