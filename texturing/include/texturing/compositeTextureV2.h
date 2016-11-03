//
// Created by Seth Parker on 12/28/15.
//
#pragma once

#include <opencv2/opencv.hpp>

#include "common/types/Texture.h"
#include "common/types/UVMap.h"
#include "common/vc_defines.h"
#include "volumepkg/volumepkg.h"

#include "texturingUtils.h"

namespace volcart
{
namespace texturing
{

class compositeTextureV2
{
public:
    struct cellInfo {
        std::vector<cv::Vec3d> Pts2D;
        std::vector<cv::Vec3d> Pts3D;
        cv::Vec3d Normal;
    };

    constexpr static size_t KD_DEFAULT_SEARCH_SIZE = 100;

    compositeTextureV2(
        ITKMesh::Pointer inputMesh,
        VolumePkg& volpkg,
        UVMap uvMap,
        double radius,
        int width,
        int height,
        CompositeOption method = CompositeOption::NonMaximumSuppression,
        DirectionOption direction = DirectionOption::Bidirectional);

    const volcart::Texture& texture() const { return _texture; };
    volcart::Texture& texture() { return _texture; };
private:
    int _process();
    int _generateCellInfo();
    cv::Vec3d _BarycentricCoord(
        const cv::Vec3d& nXYZ,
        const cv::Vec3d& nA,
        const cv::Vec3d& nB,
        const cv::Vec3d& nC);
    cv::Vec3d _CartesianCoord(
        const cv::Vec3d& nUVW,
        const cv::Vec3d& nA,
        const cv::Vec3d& nB,
        const cv::Vec3d& nC);

    // Variables
    ITKMesh::Pointer _input;
    VolumePkg& _volpkg;
    int _width;
    int _height;
    double _radius;
    CompositeOption _method;
    DirectionOption _direction;

    UVMap _uvMap;
    Texture _texture;

    std::vector<cellInfo> _cellInformation;
    ITKMesh::Pointer _cellCentroids;
    ITKPointsLocator::Pointer _kdTree;
    size_t _kdSearchSize;
};
}
}
