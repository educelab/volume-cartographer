// VC OBJ Exporter v1.0
// Created by Media Team on 6/24/15.
//
#pragma once

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>

#include "vc/core/types/Rendering.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart
{
namespace io
{

class OBJWriter
{

public:
    OBJWriter() {}
    OBJWriter(boost::filesystem::path outputPath, ITKMesh::Pointer mesh)
        : outputPath_{std::move(outputPath)}, mesh_{mesh}
    {
    }
    OBJWriter(
        boost::filesystem::path outputPath,
        ITKMesh::Pointer mesh,
        volcart::UVMap uvMap,
        cv::Mat uvImg)
        : outputPath_{std::move(outputPath)}
        , mesh_{mesh}
        , textCoords_{std::move(uvMap)}
        , texture_{std::move(uvImg)}
    {
    }

    void setPath(boost::filesystem::path path)
    {
        outputPath_ = std::move(path);
    }

    void setRendering(const volcart::Rendering& rendering);

    // Set pieces individually
    void setMesh(const ITKMesh::Pointer& mesh) { mesh_ = mesh; }
    void setUVMap(volcart::UVMap uvMap) { textCoords_ = std::move(uvMap); }
    void setTexture(const cv::Mat& uvImg) { texture_ = uvImg; }

    bool validate();  // make sure all required output parameters have been set

    boost::filesystem::path getPath() const { return outputPath_; }
    int write();
    int writeOBJ();
    int writeMTL();
    int writeTexture();

private:
    boost::filesystem::path outputPath_;  // The desired filepath. This should
                                          // include the .obj extension.
    std::ofstream outputMesh_;
    std::ofstream outputMTL_;

    std::map<uint32_t, cv::Vec3i> pointLinks_;  // Keeps track of what we know
                                              // about each point in the mesh:
                                              // [ pointID, (v, vt, vn) ]

    ITKMesh::Pointer mesh_;
    volcart::UVMap textCoords_;  // UV map for points accessed by point index
    cv::Mat texture_;            // output texture image

    int write_header_();
    int write_vertices_();
    int write_texture_coordinates_();
    int write_faces_();
};
}
}
