// VC PLY Exporter v1.0
// Created by Media Team on 10/30/15.
//
#pragma once

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

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
        : _outputPath{std::move(outputPath)}, _mesh{mesh}
    {
    }
    PLYWriter(
        boost::filesystem::path outputPath,
        ITKMesh::Pointer mesh,
        volcart::Texture texture)
        : _outputPath{std::move(outputPath)}
        , _mesh{mesh}
        , _texture{std::move(texture)}
    {
    }

    void setPath(boost::filesystem::path path)
    {
        _outputPath = std::move(path);
    }
    void setMesh(const ITKMesh::Pointer& mesh) { _mesh = mesh; }
    void setTexture(volcart::Texture texture) { _texture = std::move(texture); }

    boost::filesystem::path getPath() const { return _outputPath; }

    bool validate();  // make sure all required output parameters have been set

    int write();

private:
    boost::filesystem::path _outputPath;  // The desired filepath. This should
                                          // include the .obj extension.
    std::ofstream _outputMesh;

    ITKMesh::Pointer _mesh;
    volcart::Texture _texture;

    int _writeHeader();
    int _writeVertices();
    int _writeFaces();
};
}
}
