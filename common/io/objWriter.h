// VC OBJ Exporter v1.0
// Created by Media Team on 6/24/15.
//

#ifndef VC_IO_OBJWRITER_H
#define VC_IO_OBJWRITER_H

#include <iostream>
#include <fstream>

#include <itkMesh.h>
#include <itkTriangleCell.h>

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

namespace volcart {
namespace io {

  class objWriter {

  protected:
    // ITK typedefs to setup the mesh. This class only supports meshes used by VC software.
    typedef itk::Vector< double, 3 >                   PixelType;
    typedef itk::Mesh< PixelType, 3 >                  MeshType;
    typedef MeshType::CellType                         CellType;

    typedef MeshType::PointsContainer::ConstIterator   PointsInMeshIterator;
    typedef MeshType::CellsContainer::Iterator         CellIterator;
    typedef CellType::PointIdIterator                  PointsInCellIterator;

  public:
    objWriter();
    objWriter( std::string outputPath, itk::Mesh<PixelType, 3>::Pointer mesh );
    objWriter( std::string outputPath, itk::Mesh<PixelType, 3>::Pointer mesh, std::map<double, cv::Vec2d> uvMap, cv::Mat uvImg);

    void setPath( std::string path ) { _outputPath = path; };
    void setMesh( itk::Mesh<PixelType, 3>::Pointer mesh ) { _mesh = mesh; };
    void setUVMap( std::map<double, cv::Vec2d> uvMap ) { _textCoords = uvMap; };
    void setTexture( cv::Mat uvImg ) { _texture = uvImg; };

    bool validate(); // make sure all required output parameters have been set

    int write();
    int writeOBJ();
    int writeMTL();
    int writeTexture();

  private:
    boost::filesystem::path _outputPath; // The desired filepath. This should include the .obj extension.
    std::ofstream           _outputMesh;
    std::ofstream           _outputMTL;

    std::map<double, cv::Vec3d> _point_links;

    itk::Mesh< PixelType, 3 >::Pointer _mesh;
    std::map<double, cv::Vec2d> _textCoords; // UV map for points accessed by point index
    cv::Mat _texture; // output texture image

    int _writeHeader();
    int _writeVertices();
    int _writeTextureCoordinates();
    int _writeFaces();
  };


} // namespace io
} //namespace volcart

#endif //VC_IO_OBJWRITER_H