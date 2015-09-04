#ifndef _DEMO_CHAIN_
#define _DEMO_CHAIN_

#define DEFAULT_OFFSET -1

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "field.h"
#include "particle.h"

// this is similar to the chain class in structureTensor
// it tries to find the next position of the particles
// based on slices orthogonal to the chain

namespace DEMO {

class Chain {
public:
    Chain(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, VolumePkg *, int, int, int=1);

    void step(Field &);

    bool isMoving();

    void debug(bool=false);

    pcl::PointCloud<pcl::PointXYZRGB> orderedPCD();

private:
    // History of the chain at each iteration
    std::list<std::vector<Particle>> _history;
    VolumePkg *_volpkg;
    int _updateCount;

    // "reslicing" happens when we update the normals
    // we have to reslice every iteration regardless
    // since offets are from the center of the slice
    int _stepsBeforeReslice;
    std::vector<cv::Vec3f> _savedNormals;

    // -- Chain Size Information -- //
    uint64_t _chainLength; // Number of particles in the chain & width of output PCD
    uint32_t _realIterationsCount; // Height of the output PCD To-Do: Do we need this?
    uint32_t _startIdx; // Starting slice index
    int32_t _targetIdx; // Target slice index
    int32_t _threshold; // To-Do: What is this for now? We may not need this.

    void updateNormal(uint64_t);
};

}

#endif
