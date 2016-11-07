// UDataManipulateUtils.h
// Chao Du 2014 Dec
#pragma once

#include <qimage.h>

#include <opencv2/opencv.hpp>

namespace ChaoVis
{

bool SplitVertexAndElementBuffer(
    int nVertexNum,
    int nFaceNum,
    const int* nElementBufferTmp,  // constant data, not constant pointer
    unsigned short*** nElementBufferData,
    const float* nVertexBufferTmp,
    float*** nVertexBufferData,
    const float* nUVBufferTmp,
    float*** nUVBufferData,
    int** nElementBufferSize,
    int** nVertexBufferSize,
    int** nUVBufferSize,
    int* nElementArrayNum);

cv::Mat QImage2Mat(const QImage& nSrc);

QImage Mat2QImage(const cv::Mat& nSrc);

}  // namespace ChaoVis
