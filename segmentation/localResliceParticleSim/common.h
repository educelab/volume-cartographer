#pragma once

#ifndef _VC_COMMON_H_
#define _VC_COMMON_H_

#include <vector>
#include <tuple>

#include <opencv2/opencv.hpp>

namespace volcart {

namespace segmentation {

#define BGR_RED     cv::Scalar(0,    0   , 0xFF)
#define BGR_GREEN   cv::Scalar(0,    0xFF, 0   )
#define BGR_BLUE    cv::Scalar(0xFF, 0,    0   )
#define BGR_YELLOW  cv::Scalar(0,    0xFF, 0xFF)
#define BGR_MAGENTA cv::Scalar(0xFF, 0,    0xFF)

#if !defined(VC_INDEX_X) && !defined(VC_INDEX_Y) && !defined(VC_INDEX_Z)
#define VC_INDEX_X 0
#define VC_INDEX_Y 1
#define VC_INDEX_Z 2
#endif

#define VC_DIRECTION_K cv::Vec3d(0.0, 0.0, 1.0)

using IndexDistPair = std::pair<int32_t, double>;
using IndexDistPairVec = typename std::vector<IndexDistPair>;
using Positions = std::vector<cv::Vec3d>;

enum Direction {
    kLeft = -1,
    kNone = 0,
    kRight = 1,
    kDefault = 10
};

}

}

#endif //VC_COMMON_H
