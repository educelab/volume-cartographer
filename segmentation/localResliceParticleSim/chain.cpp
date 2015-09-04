#include "chain.h"


DEMO::Chain::Chain(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath, VolumePkg* volpkg, int threshold, int endOffset,
                   int stepsBeforeReslice) :
    _volpkg(volpkg), _updateCount(0), _threshold(threshold), _stepsBeforeReslice(stepsBeforeReslice) {
    // Convert the point cloud segPath into a vector of Particles
    std::vector<Particle> initChain;
    for (auto path : *segPath) {
        initChain.push_back(Particle(path.x, path.y, path.z));
    }

    // Add starting chain to _history and setup other parameters
    _history.push_front(initChain);
    _chainLength = initChain.size();

    // normals are updated every iteration by default
    _savedNormals = std::vector<cv::Vec3f>(_chainLength, cv::Vec3f(0, 0, 0));

    // Find the lowest slice index in the starting chain
    _startIdx = uint32_t(_history.front()[0](VC_INDEX_Z));
    for (int i = 0; i < _chainLength; ++i) {
        if (_history.front()[i](VC_INDEX_Z) < _startIdx) {
            _startIdx = uint32_t(_history.front()[i](VC_INDEX_Z));
        }
    }

    // Set the slice index we will end at
    // If user does not define endOffset, target index == last slice with a surface normal file
    _targetIdx = ((endOffset == DEFAULT_OFFSET)
                     ? (volpkg->getNumberOfSlices() - 3U) // Account for zero-indexing and slices lost in calculating normal vector
                     : (_startIdx + endOffset));

    // Set _realIterationsCount based on starting index, target index, and how frequently we want to sample the segmentation
    _realIterationsCount = uint32_t(ceil(((_targetIdx - _startIdx) + 1) / _threshold));
}

void DEMO::Chain::updateNormal(uint64_t i) {
    auto updatingChain = _history.front();
    cv::Vec3f tangent = updatingChain[i+1] - updatingChain[i-1];
    _savedNormals[i] = tangent.cross(VC_DIRECTION_K);
}

// This function defines how particles are updated
void DEMO::Chain::step(DEMO::Field& field) {
    // Pull the most recent iteration from _history
    auto update_chain = _history.front();
    std::vector<cv::Vec3f> force_vector(_chainLength, cv::Vec3f(0, 0, 0));

    for (int i = 0; i < _chainLength; ++i) {
        if (update_chain[i].isStopped())
            continue;

        // update normals every _stepsBeforeReslice steps
        if (_updateCount % _stepsBeforeReslice == 0) {
            // pretend that the normals at the end of the chain
            // are the same as the ones adjacent
            if (i == 0) {
                updateNormal(1);
            } else if (i == _chainLength - 1) {
                updateNormal(_chainLength - 2);
            } else {
                updateNormal(uint64_t(i));
            }
        }

        // reslice and find next position
        Slice s = field.reslice(update_chain[i].position(), _savedNormals[i], VC_DIRECTION_K);

        if (i == 32) {
            s.debugDraw(DEBUG_DRAW_CENTER);
            s.debugAnalysis();
        }

        force_vector[i] += (s.findNextPosition() - update_chain[i].position());
    }

    // update the chain
    for (int i = 0; i < _chainLength; ++i) {
        update_chain[i] += force_vector[i];
        if (floor(update_chain[i](VC_INDEX_Z)) >= _targetIdx) {
            update_chain[i].stop();
        }
    }

    // Add the modified chain back to _history
    _updateCount++;
    _history.push_front(update_chain);
}

// Returns true if any Particle in the chain is still moving
bool DEMO::Chain::isMoving() {
    bool result = true;
    for (int i = 0; i < _chainLength; ++i) {
        result &= _history.front()[i].isStopped();
    }
    return !result;
}

// draw a debug window with an option to write to disk
void DEMO::Chain::debug(bool saveOutput) {
    std::vector<Particle> recent = _history.front();
    int z_index = recent[0](VC_INDEX_Z);

    cv::Mat debug = _volpkg->getSliceData(z_index);
    debug *= 1. / 255;
    debug.convertTo(debug, CV_8UC3);
    cvtColor(debug, debug, CV_GRAY2BGR);

    // draw circles on the debug window for each point
    for (int i = 0; i < recent.size(); ++i) {
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
pcl::PointCloud<pcl::PointXYZRGB> DEMO::Chain::orderedPCD() {
    // Allocate space for one row of the output cloud
    std::vector<pcl::PointXYZRGB> storage_row;
    for (int i = 0; i < _chainLength; ++i) {
        pcl::PointXYZRGB point;
        point.z = -1; // To-Do: Make this a constant
        storage_row.push_back(point);
    }

    // Allocate space for all rows of the output cloud
    // storage will represent the cloud with 2D indexes
    std::vector<std::vector<pcl::PointXYZRGB>> storage;
    for (int i = 0; i < _realIterationsCount; ++i) {
        storage.push_back(storage_row);
    }

    // Give the output points an abitrary color. *To-Do: This is not used ever.
    uint32_t COLOR = 0x00777777; // grey in PCL's packed RGB representation

    // Push each point in _history into its ordered position in storage if it passes the distance threshold
    for (auto v : _history) {
        // Add each Particle in the row into storage at the correct position
        // Note: This is where we convert the internal cloud's coordinate ordering back to volume ordering
        for (int i = 0; i < _chainLength; ++i) {
            int currentCell = int(((v[i](VC_INDEX_Z)) - _startIdx / _threshold)); //TODO: Something seems wrong here.
            pcl::PointXYZRGB point;
            point.x = v[i](VC_INDEX_X);
            point.y = v[i](VC_INDEX_Y);
            point.z = v[i](VC_INDEX_Z);
            point.rgb = *(float *) &COLOR;
            storage[currentCell][i] = point;
        }

        // Move points out of storage into the point cloud
        pcl::PointCloud <pcl::PointXYZRGB> cloud;
        cloud.height = _realIterationsCount;
        cloud.width = uint32_t(_chainLength);
        cloud.points.resize(cloud.height * cloud.width);
        for (int i = 0; i < cloud.height; ++i) {
            for (int j = 0; j < cloud.width; ++j) {
                cloud.points[j + (i * cloud.width)] = storage[i][j];
            }
        }
        return cloud;
    }
}
