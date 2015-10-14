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

Particle Chain::at(const uint32_t idx) const
{
    return particles_.at(idx);
}

const int32_t Chain::zIndex(void) const
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
std::pair<std::vector<Direction>, std::vector<cv::Vec3f>> Chain::stepAll() const
{
    auto positions = std::vector<cv::Vec3f>(particleCount_);
    auto directions = std::vector<Direction>(particleCount_);
    for (auto i = 0; i < particleCount_; ++i) {
        std::tie(directions[i], positions[i]) = step(i);
    }
    return std::make_pair<decltype(directions), decltype(positions)>(directions, positions);
}

cv::Vec3f Chain::calculateNormal(const uint32_t index) const
{
    // Create N x 3 matrix from chain
    auto matChain = cv::Mat(particles_.size(), 2, CV_32F);
    auto matChainZ = cv::Mat(particles_.size(), 1, CV_32F);
    for (auto i = 0; i < particles_.size(); ++i) {
        matChain.at<float>(i, VC_INDEX_X) = particles_[i].position()(VC_INDEX_X);
        matChain.at<float>(i, VC_INDEX_Y) = particles_[i].position()(VC_INDEX_Y);
        matChainZ.at<float>(i, 0) = particles_[i].position()(VC_INDEX_Z);
    }
    auto mean = cv::mean(matChainZ)(0);

    // Change indexing if we're at the front or back particle
    int32_t i = index;
    if (index == 0) {
        i = 1;
    } else if (index == particles_.size() - 1) {
        i = particleCount_ - 2;
    }

    auto tangent = cv::Point2f(matChain.at<float>(i+1) - matChain.at<float>(i-1));
    auto tanVec = cv::Vec3f(tangent.x, tangent.y, mean);
    return tanVec.cross(VC_DIRECTION_K);
}

// Step an individual particle with optional direction and drift constraints
std::pair<Direction, cv::Vec3f> Chain::step(const int32_t index, const Direction dirConstraint, double maxDrift) const
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
    using IndexDistPair = std::pair<uint32_t, double>;
    std::sort(maxima.begin(), maxima.end(), [center](IndexDistPair lhs, IndexDistPair rhs) {
        auto x = center.x;
        auto ldist = std::abs(int32_t(lhs.first - x));
        auto rdist = std::abs(int32_t(rhs.first - x));
        return ldist < rdist;
    });

    // Convert from pixel space to voxel space to enforce constraints (if necessary)
    using DirPosPair = std::pair<Direction, cv::Vec3f>;
    auto voxelMaxima = std::vector<DirPosPair>(maxima.size());
    std::transform(maxima.begin(), maxima.end(), std::back_inserter(voxelMaxima), [reslice, center](IndexDistPair p) {
        Direction d = Direction::kNone;
        if (p.first - center.y > 0) {
            d = Direction::kLeft;
        } else if (p.first - center.y < 0) {
            d = Direction::kRight;
        } else {
            d = Direction::kNone;
        }
        auto voxel = reslice.sliceCoordToVoxelCoord(cv::Point(p.first, center.y + lookaheadDepth));
        return std::make_pair(d, voxel);
    });

    // Remove any pairs that don't satisfy the direction constraint (default behavior is to not remove any)
    // XXX assumes that dirConstraint is either Direction::kLeft or Direction::kRight
    std::remove_if(maxima.begin(), maxima.end(), [center, dirConstraint](IndexDistPair p) {
        return (p.first != Direction::kNone) && (dirConstraint != p.first);
    });

    // Remove any pairs that don't satisfy the position constraint
    if (maxDrift != kDefaultMaxDrift) {
        std::remove_if(voxelMaxima.begin(), voxelMaxima.end(), [maxDrift, currentParticle](DirPosPair p) {
            return cv::norm(p.second - currentParticle.position()) > maxDrift;
        });
    }

    // XXX What do we do if there's nothing left in the voxelMaxima at this point?

    // Find next point in slice space
    return voxelMaxima.at(0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Chain::drawChainOnSlice(std::vector<Particle> v) {
    auto zidx = v[0](VC_INDEX_Z);
    auto debug = _volpkg.getSliceData(zidx);
    debug /= 255.0;
    debug.convertTo(debug, CV_8UC3);
    cvtColor(debug, debug, CV_GRAY2BGR);

    // draw circles on the debug window for each point
    for (uint32_t i = 0; i < v.size(); ++i) {
        cv::Point position(v[i](VC_INDEX_X), v[i](VC_INDEX_Y));
        if (i == 32)
            circle(debug, position, 2, cv::Scalar(0, 255, 255), -1);
        else
            circle(debug, position, 2, cv::Scalar(0, 255, 0), -1);
    }

    namedWindow("DEBUG CHAIN", cv::WINDOW_NORMAL);
    imshow("DEBUG CHAIN", debug);
}

// draw a debug window with an option to write to disk
void Chain::debug(bool saveOutput) {
    std::vector<Particle> recent = _history.front();
    int z_index = recent[0](VC_INDEX_Z);

    cv::Mat debug = _volpkg.getSliceData(z_index);
    debug *= 1. / 255;
    debug.convertTo(debug, CV_8UC3);
    cvtColor(debug, debug, CV_GRAY2BGR);

    // draw circles on the debug window for each point
    for (uint32_t i = 0; i < recent.size(); ++i) {
        cv::Point position(recent[i](VC_INDEX_X), recent[i](VC_INDEX_Y));
        if (i == 32)
            circle(debug, position, 2, cv::Scalar(0, 255, 255), -1);
        else
            circle(debug, position, 2, cv::Scalar(0, 255, 0), -1);
    }

    namedWindow("DEBUG CHAIN", cv::WINDOW_AUTOSIZE);
    imshow("DEBUG CHAIN", debug);

    // option to save output to disk
    if (saveOutput) {
        std::stringstream ss;
        ss << "debug_chain_" << std::setw(3) << std::setfill('0') << _updateCount << ".tif";
        cv::imwrite(ss.str(), debug);
    }

    cv::waitKey(0);
}

// Convert Chain's _history to an ordered Point Cloud object
pcl::PointCloud<pcl::PointXYZRGB> Chain::orderedPCD() {
    // Allocate space for one row of the output cloud
    std::vector<pcl::PointXYZRGB> storage_row;
    for (uint32_t i = 0; i < _chainLength; ++i) {
        pcl::PointXYZRGB point;
        point.z = -1; // To-Do: Make this a constant
        storage_row.push_back(point);
    }

    // Allocate space for all rows of the output cloud
    // storage will represent the cloud with 2D indexes
    std::vector<std::vector<pcl::PointXYZRGB>> storage;
    for (uint32_t i = 0; i < _realIterationsCount; ++i) {
        storage.push_back(storage_row);
    }

    // Give the output points an abitrary color. *To-Do: This is not used ever.
    uint32_t COLOR = 0x00777777; // grey in PCL's packed RGB representation

    // Push each point in _history into its ordered position in storage if it passes the distance threshold
    for (auto v : _history) {
        // Add each Particle in the row into storage at the correct position
        // Note: This is where we convert the internal cloud's coordinate ordering back to volume ordering
        for (uint32_t i = 0; i < _chainLength; ++i) {
            int currentCell = int(((v[i](VC_INDEX_Z)) - _startIdx / _stepSize)); //TODO: Something seems wrong here.
            pcl::PointXYZRGB point;
            point.x = v[i](VC_INDEX_X);
            point.y = v[i](VC_INDEX_Y);
            point.z = v[i](VC_INDEX_Z);
            point.rgb = *(float *) &COLOR;
            storage[currentCell][i] = point;
        }
    }

    // Move points out of storage into the point cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.height = _realIterationsCount;
    cloud.width = uint32_t(_chainLength);
    cloud.points.resize(cloud.height * cloud.width);
    for (uint32_t i = 0; i < cloud.height; ++i) {
        for (uint32_t j = 0; j < cloud.width; ++j) {
            cloud.points[j + (i * cloud.width)] = storage[i][j];
        }
    }
    return cloud;
}
