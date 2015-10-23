#include "chainmesh.h"

using namespace volcart::segmentation;

ChainMesh::ChainMesh() : nextRow_(0), width_(0), height_(0) { }

// NOTE: OpenCV does rows first, then columns, so need to flip the order of
// width/height when passing to Mat constructor
ChainMesh::ChainMesh(const int32_t width, const int32_t height) :
        nextRow_(0), width_(width), height_(height)
{
    positions_ = cv::Mat(height, width, CV_64FC3);
}

// Pushes a chain back into the ChainMesh
void ChainMesh::addChain(Chain row)
{
    for (int32_t i = 0; i < row.size(); ++i) {
        positions_.at<cv::Vec3f>(nextRow_, i) = cv::Vec3f(row.at(i).position());
    }
    nextRow_++;
}

// Resets size of matrix. NOTE: Deletes current matrix in chainmesh
void ChainMesh::setSize(const int32_t width, const int32_t height)
{
    positions_ = cv::Mat(height, width, CV_64FC3);
}

// Export the mesh as an ordered PCD
// Note: Need to export as PointXYZRGB since that's how it's stored in VolumePkg
pcl::PointCloud<pcl::PointXYZRGB> ChainMesh::exportAsPCD() const {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    // Set size. Since this is unordered (for now...) just set the width to be the
    // number of points and the height (by convention) is set to 1
    cloud.width = uint32_t(positions_.cols * positions_.rows);
    cloud.height = 1;
    cloud.is_dense = false;

    for (int32_t i = 0; i < positions_.rows; ++i) {
        for (int32_t j = 0; j < positions_.cols; ++j) {
            pcl::PointXYZRGB p;
            p.x = positions_.at<cv::Vec3f>(i, j)(VC_INDEX_X);
            p.y = positions_.at<cv::Vec3f>(i, j)(VC_INDEX_Y);
            p.z = positions_.at<cv::Vec3f>(i, j)(VC_INDEX_Z);
            p.r = 0xFF;
            p.g = 0xFF;
            p.b = 0xFF;
            cloud.push_back(p);
        }
    }
    return cloud;
}
