//
// Created by Sean Karlage on 10/2/15.
//

#include "reslice.h"

Reslice::Reslice(cv::Mat data, cv::Vec3f origin, cv::Vec3f xvec, cv::Vec3f yvec) :
        sliceData_(data), origin_(origin), xvec_(xvec), yvec_(yvec) {
}

cv::Vec3f Reslice::sliceCoordToVoxelCoord(cv::Point sliceCoords) {
    return origin_ + (sliceCoords.x * xvec_ + sliceCoords.y * yvec_);
}
