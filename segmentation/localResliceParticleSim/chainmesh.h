#pragma once

#ifndef _VC_SEGMENTATION_CHAINMESH_H_
#define _VC_SEGMENTATION_CHAINMESH_H_

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "chain.h"

namespace volcart {

namespace segmentation {

class ChainMesh {
public:
    ChainMesh(const int32_t width, const int32_t height);

    pcl::PointCloud<pcl::PointXYZRGB> exportAsPointCloud();

    void addChain(const Chain& c);

    void addPositions(const VoxelVec& ps);

    cv::Mat positions() const { return positions_; }

    int32_t width() const { return width_; }

    int32_t height() const { return height_; }

private:
    cv::Mat positions_;
    int32_t nextRow_;
    int32_t width_;
    int32_t height_;

};

}

}

#endif //VC_CHAINMESH_H
