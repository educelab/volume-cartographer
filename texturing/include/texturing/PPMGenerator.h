#pragma once

#include <opencv2/core.hpp>

#include "core/types/PerPixelMap.h"
#include "core/types/UVMap.h"
#include "core/vc_defines.h"

namespace volcart
{
namespace texturing
{

class PPMGenerator
{
public:
    // Constructors/Destructors
    PPMGenerator() : _height(0), _width(0){};
    PPMGenerator(size_t h, size_t w) : _height(h), _width(w){};

    // Set/Get Parameters
    void setMesh(ITKMesh::Pointer m) { _inputMesh = m; };
    void setUVMap(const UVMap& u) { _uvMap = u; };
    void setDimensions(size_t h, size_t w);

    // Run
    void compute();

    // Output
    const PerPixelMap& getPPM() const { return _ppm; };
    PerPixelMap& getPPM() { return _ppm; };

private:
    struct CellInfo {
        void reset()
        {
            Pts2D.clear();
            Pts3D.clear();
        }
        std::vector<cv::Vec3d> Pts2D;
        std::vector<cv::Vec3d> Pts3D;
        cv::Vec3d Normal;
    };

    // Helpers
    void _generateCentroidMesh();
    void _generatePPM();
    cv::Vec3d _BarycentricCoord(
        const cv::Vec3d nXYZ,
        const cv::Vec3d nA,
        const cv::Vec3d nB,
        const cv::Vec3d nC);
    cv::Vec3d _CartesianCoord(
        const cv::Vec3d nUVW,
        const cv::Vec3d nA,
        const cv::Vec3d nB,
        const cv::Vec3d nC);

    // Data members
    ITKMesh::Pointer _inputMesh;
    ITKMesh::Pointer _centroidMesh;
    std::vector<CellInfo> _cellInformation;
    UVMap _uvMap;
    PerPixelMap _ppm;

    size_t _width;
    size_t _height;

    double _progress;
};
}
}  // namespace volcart
