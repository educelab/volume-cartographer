#pragma once

#include <opencv2/core.hpp>

#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/UVMap.hpp"
#include "vc/core/vc_defines.hpp"

namespace volcart
{
namespace texturing
{

class PPMGenerator
{
public:
    // Constructors/Destructors
    PPMGenerator() : width_{0}, height_{0} {}
    PPMGenerator(size_t h, size_t w) : width_{w}, height_{h} {}

    // Set/Get Parameters
    void setMesh(const ITKMesh::Pointer& m) { inputMesh_ = m; }
    void setUVMap(const UVMap& u) { uvMap_ = u; }
    void setDimensions(size_t h, size_t w);

    // Run
    PerPixelMap& compute();

    // Output
    const PerPixelMap& getPPM() const { return ppm_; }
    PerPixelMap& getPPM() { return ppm_; }

private:
    struct CellInfo {
        void reset()
        {
            pts2D.clear();
            pts3D.clear();
        }
        std::vector<cv::Vec3d> pts2D;
        std::vector<cv::Vec3d> pts3D;
        cv::Vec3d normal;
    };

    // Helpers
    void generate_centroid_mesh_();
    void generate_ppm_();
    cv::Vec3d barycentric_coord_(
        const cv::Vec3d& nXYZ,
        const cv::Vec3d& nA,
        const cv::Vec3d& nB,
        const cv::Vec3d& nC);
    cv::Vec3d cartesian_coord_(
        const cv::Vec3d& nUVW,
        const cv::Vec3d& nA,
        const cv::Vec3d& nB,
        const cv::Vec3d& nC);

    // Data members
    ITKMesh::Pointer inputMesh_;
    ITKMesh::Pointer centroidMesh_;
    std::vector<CellInfo> cellInformation_;
    UVMap uvMap_;
    PerPixelMap ppm_;

    size_t width_;
    size_t height_;

    double progress_;
};
}  // namespace texturing
}  // namespace volcart
