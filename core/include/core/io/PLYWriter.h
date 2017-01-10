// VC PLY Exporter v1.0
// Created by Media Team on 10/30/15.

#pragma once

#include <fstream>

#include <boost/filesystem.hpp>

#include "core/types/Texture.h"
#include "core/vc_defines.h"

namespace volcart
{
namespace io
{

class PLYWriter
{
public:
    PLYWriter() {}
    PLYWriter(boost::filesystem::path outputPath, ITKMesh::Pointer mesh)
        : outputPath_{std::move(outputPath)}, mesh_{mesh}
    {
    }
    PLYWriter(
        boost::filesystem::path outputPath,
        ITKMesh::Pointer mesh,
        volcart::Texture texture)
        : outputPath_{std::move(outputPath)}
        , mesh_{mesh}
        , texture_{std::move(texture)}
    {
    }

    void setPath(boost::filesystem::path path)
    {
        outputPath_ = std::move(path);
    }
    void setMesh(const ITKMesh::Pointer& mesh) { mesh_ = mesh; }
    void setTexture(volcart::Texture texture) { texture_ = std::move(texture); }

    boost::filesystem::path getPath() const { return outputPath_; }

    // make sure all required output parameters have been set
    bool validate();

    int write();

private:
    boost::filesystem::path outputPath_;
    std::ofstream outputMesh_;
    ITKMesh::Pointer mesh_;
    volcart::Texture texture_;
    int writeHeader_();
    int writeVertices_();
    int writeFaces_();
};
}
}
