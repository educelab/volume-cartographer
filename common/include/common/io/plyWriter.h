// VC PLY Exporter v1.0
// Created by Media Team on 10/30/15.
//
#pragma once

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/opencv.hpp>

#include "common/types/Texture.h"
#include "common/vc_defines.h"

namespace volcart
{
namespace io
{
class plyWriter
{
public:
    plyWriter(){};
    plyWriter(boost::filesystem::path outputPath, VC_MeshType::Pointer mesh);
    plyWriter(boost::filesystem::path outputPath,
              VC_MeshType::Pointer mesh,
              volcart::Texture texture);

    void setPath(boost::filesystem::path path) { _outputPath = path; };
    void setMesh(VC_MeshType::Pointer mesh) { _mesh = mesh; };
    void setTexture(volcart::Texture texture) { _texture = texture; };

    boost::filesystem::path getPath() const { return _outputPath; }

    bool validate();  // make sure all required output parameters have been set

    int write();

private:
    boost::filesystem::path _outputPath;  // The desired filepath. This should
                                          // include the .obj extension.
    std::ofstream _outputMesh;

    VC_MeshType::Pointer _mesh;
    volcart::Texture _texture;

    int _writeHeader();
    int _writeVertices();
    int _writeFaces();
};
}
}
