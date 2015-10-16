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

    ChainMesh(const int32_t width, const int32_t height);

    pcl::PointCloud<pcl::PointXYZRGB> exportAsPCD() const;

    void addChain(Chain c);

    void setSize(const int32_t width, const int32_t height);

    cv::Mat positions() const { return positions_; }

    int32_t width() const { return width_; }

    int32_t height() const { return height_; }

private:
    cv::Mat positions_;
    int32_t width_;
    int32_t height_;

};

}

}

#endif //VC_CHAINMESH_H
