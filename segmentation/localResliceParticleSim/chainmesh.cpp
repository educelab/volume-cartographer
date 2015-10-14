#include "chainmesh.h"

using namespace volcart::segmentation;

ChainMesh::ChainMesh() : height_(0), width_(0) { }

// NOTE: OpenCV does rows first, then columns, so need to flip the order of width/height when passing to Mat constructor
ChainMesh::ChainMesh(const uint32_t width, const uint32_t height) : width_(width), height_(height) {
    positions_ = cv::Mat(height, width, CV_64FC3);
}

// Pushes a chain back into the ChainMesh
void ChainMesh::addChain(Chain row) {
    for (auto i = 0; i < row.size(); ++i) {
        for (auto component = VC_INDEX_X; component <= VC_INDEX_Z; ++component) {
            positions_.at(row.zIndex(), i)(component) = row.at(i);
        }
    }
}

// Resets size of matrix. NOTE: Deletes current matrix in chainmesh
void ChainMesh::setSize(const uint32_t width, const uint32_t height)
{
    positions_ = cv::Mat(height, width, CV_64FC3);
}

// Export the mesh as an ordered PCD
pcl::PointCloud<pcl::PointXYZRGB> exportAsOrderedPCD() const {
    // TODO: Implement
}
