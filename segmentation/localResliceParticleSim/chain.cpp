#include <algorithm>
#include <functional>

#include "chain.h"

using namespace volcart::segmentation;

Chain::Chain(VolumePkg& volpkg) : volpkg_(volpkg), particleVecIsSet_(false) {
    auto segmentationPath = volpkg_.openCloud();
    for (auto path : *segmentationPath) {
        particles_.push_back(Particle(path.x, path.y, path.z));
    }
}

Chain::Chain(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath, VolumePkg& volpkg, int stepSize, int endOffset,
                   int stepsBeforeReslice) :
    _volpkg(volpkg), _updateCount(0), _stepsBeforeReslice(stepsBeforeReslice), _stepSize(stepSize) {
    // Convert the point cloud segPath into a vector of Particles
    std::vector<Particle> initChain;
    for (auto path : *segPath) {
        initChain.push_back(Particle(path.x, path.y, path.z));
    }

    // Add starting chain to _history and setup other parameters
    _history.push_front(initChain);
    _chainLength = initChain.size();

    // Saved particle normals for when they're updated every N slices
    _savedNormals = std::vector<cv::Vec3f>(_chainLength, cv::Vec3f(0, 0, 0));

    // Find the lowest slice index in the starting chain
    auto minParticle = *std::min_element(initChain.begin(), initChain.end(),
                                  [](const Particle& a, const Particle& b) { return a(VC_INDEX_Z) < b(VC_INDEX_Z); });
    _startIdx = uint32_t(minParticle(VC_INDEX_Z));

    // Set the slice index we will end at
    // If user does not define endOffset, target index == last slice with a surface normal file
    _targetIdx = ((endOffset == DEFAULT_OFFSET)
                     ? (volpkg.getNumberOfSlices() - 3U) // Account for zero-indexing and slices lost in calculating normal vector
                     : (_startIdx + endOffset));

    // Set _realIterationsCount based on starting index, target index, and how frequently we want to sample the segmentation
    _realIterationsCount = uint32_t(ceil(((_targetIdx - _startIdx) + 1) / _stepSize));
}

cv::Vec3f Chain::calculateNormal(uint64_t i, std::vector<Particle> prevChain) {
    // Create N x 3 matrix from chain
    auto matChain = cv::Mat(prevChain.size(), 2, CV_32F);
    auto matChainZ = cv::Mat(prevChain.size(), 1, CV_32F);
    for (auto i = 0; i < prevChain.size(); ++i) {
        matChain.at<float>(i, VC_INDEX_X) = prevChain[i].position()(VC_INDEX_X);
        matChain.at<float>(i, VC_INDEX_Y) = prevChain[i].position()(VC_INDEX_Y);
        matChainZ.at<float>(i, 0) = prevChain[i].position()(VC_INDEX_Z);
    }
    auto mean = cv::mean(matChainZ)(0);
    auto tangent = cv::Point2f(matChain.at<float>(i+1) - matChain.at<float>(i-1));
    auto tanVec = cv::Vec3f(tangent.x, tangent.y, mean);
    return tanVec.cross(VC_DIRECTION_K);
}

// This function defines how particles are updated
void Chain::step(Field& field) {
    // Pull the most recent iteration from _history
    auto updateChain = _history.front();
    drawChainOnSlice(updateChain);
    std::vector<cv::Vec3f> forceVector(_chainLength, cv::Vec3f(0, 0, 0));
    //std::for_each(updateChain.begin(), updateChain.end(), [](Particle p) { std::cout << p << ", "; });

    for (uint32_t i = 0; i < _chainLength; ++i) {
        if (!updateChain[i].isMoving())
            continue;

        // update normals every _stepsBeforeReslice steps
        //if (_updateCount % _stepsBeforeReslice == 0) {
            // pretend that the normals at the end of the chain are the same as the ones adjacent
            if (i == 0) {
                _savedNormals[i] = calculateNormal(1, updateChain);
            } else if (i == _chainLength - 1) {
                _savedNormals[i] = calculateNormal(_chainLength-2, updateChain);
            } else {
                _savedNormals[i] = calculateNormal(i, updateChain);
            }
        //}

        // reslice and find next position
        Slice s = field.reslice(updateChain[i].position(), _savedNormals[i], VC_DIRECTION_K);

        if (i == 29) {
            s.debugDraw(DEBUG_DRAW_CENTER);
            s.drawSliceAndCenter();
        }
        forceVector[i] += (s.findNextPosition() - updateChain[i].position());
    }

    // update the chain
    for (uint32_t i = 0; i < _chainLength; ++i) {
        updateChain[i] += forceVector[i];
        if (floor(updateChain[i](VC_INDEX_Z)) >= _targetIdx) {
            updateChain[i].stop();
        }
    }

    // Add the modified chain back to _history
    //std::for_each(updateChain.begin(), updateChain.end(), [](Particle p) { std::cout << p << ", "; });
    _updateCount++;
    _history.push_front(updateChain);
    cv::waitKey(0);
}

// Returns true if any Particle in the chain is still moving
bool Chain::hasMovingParticle() {
    return std::any_of(_history.front().begin(), _history.front().end(), [](Particle p) { return p.isMoving(); });
}

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

// Converts the chain (vector<Particle>) to a cv::Mat for more efficient computation
// Caches conversion
cv::Mat Chain::particleChainToMat() {
    if (particleVecIsSet_) {
        return particleVec_;
    }
    particleVec_ = cv::Mat(particles_.size(), 3, CV_32F);
    for (int32_t i = 0; i < particles_.size(); ++i) {
        particleVec_.at<cv::Vec3f>(i) = particles_.at(i).position();
    }
    return particleVec_;
}

// Calculuates the order'th derivative of the chain
cv::Mat Chain::deriv(uint32_t order) {
    auto particleVec = particleChainToMat();
    cv::Mat d;
    cv::Sobel(particleVec, d, CV_32F, 0, order, 1);
    return d;
}

double Chain::tension() {
    auto firstDeriv = deriv(1);
    return cv::norm(firstDeriv, cv::NORM_L2SQR);
}

double Chain::stiffness() {
    auto secondDeriv = deriv(2);
    return cv::norm(secondDeriv, cv::NORM_L2SQR);
}
