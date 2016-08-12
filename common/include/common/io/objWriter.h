// VC OBJ Exporter v1.0
// Created by Media Team on 6/24/15.
//

#ifndef VC_IO_OBJWRITER_H
#define VC_IO_OBJWRITER_H

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "common/vc_defines.h"
#include "common/types/UVMap.h"
#include "common/types/Rendering.h"

namespace volcart {
namespace io {

  class objWriter {

  public:
    objWriter();
    objWriter( boost::filesystem::path outputPath, VC_MeshType::Pointer mesh );
    objWriter( boost::filesystem::path outputPath, VC_MeshType::Pointer mesh, volcart::UVMap uvMap, cv::Mat uvImg);

    void setPath( boost::filesystem::path path ) { _outputPath = path; };

    void setRendering( volcart::Rendering rendering );

    // Set pieces individually
    void setMesh( VC_MeshType::Pointer mesh ) { _mesh = mesh; };
    void setUVMap( volcart::UVMap uvMap ) { _textCoords = uvMap; };
    void setTexture( cv::Mat uvImg ) { _texture = uvImg; };

    bool validate(); // make sure all required output parameters have been set

    boost::filesystem::path getPath() const {return _outputPath;}
    int write();
    int writeOBJ();
    int writeMTL();
    int writeTexture();

  private:
    boost::filesystem::path _outputPath; // The desired filepath. This should include the .obj extension.
    std::ofstream           _outputMesh;
    std::ofstream           _outputMTL;

    std::map<double, cv::Vec3d> _point_links; // Keeps track of what we know about each point in the mesh: [ pointID, (v, vt, vn) ]

    VC_MeshType::Pointer _mesh;
    volcart::UVMap _textCoords; // UV map for points accessed by point index
    cv::Mat _texture; // output texture image

    int _writeHeader();
    int _writeVertices();
    int _writeTextureCoordinates();
    int _writeFaces();
  };


} // namespace io
} //namespace volcart

#endif //VC_IO_OBJWRITER_H
