#pragma once

#ifndef VC_CHAINMESH_H
#define VC_CHAINMESH_H

#include <opencv2/core/mat.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "chain.h"

namespace volcart {

namespace segmentation {

class ChainMesh {
public:
    ChainMesh();

    ChainMesh(const uint32_t width, const uint32_t height);

    pcl::PointCloud<pcl::PointXYZRGB> exportAsOrderedPCD() const;

    void addChain(Chain c);

    void setSize(const uint32_t width, const uint32_t height);

    cv::Mat positions() const { return positions_; }

    uint32_t width() const { return width_; }

    uint32_t height() const { return height_; }

private:
    cv::Mat positions_;
    uint32_t width_;
    uint32_t height_;

};

}

}

#endif //VC_CHAINMESH_H
