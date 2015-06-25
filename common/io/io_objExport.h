//
// Created by Media Team on 6/24/15.
//

#ifndef VC_IO_OBJEXPORT_H
#define VC_IO_OBJEXPORT_H

#include <iostream>
#include <fstream>
#include <unordered_map>

#include <itkMesh.h>
#include <itkTriangleCell.h>

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

namespace volcart {


namespace io {

  class objWriter {
  public:
    objWriter( std::string outputPath, itk::Mesh<::itk::Vector<double, 3>, 3>::Pointer mesh );
    objWriter( std::string outputPath, itk::Mesh<::itk::Vector<double, 3>, 3>::Pointer mesh, std::unordered_map<unsigned long, cv::Vec2d> uvMap, cv::Mat uvImg);
    int write();
    int writeMesh();
    int writeMTL();
    int writeTexture();

  protected:

    typedef itk::Vector< double, 3 >            PixelType;
    typedef itk::Mesh< PixelType, 3 >           MeshType;
    typedef MeshType::CellType                  CellType;
    typedef itk::TriangleCell< CellType >       TriangleType;

    typedef MeshType::PointsContainer::ConstIterator   PointsInMeshIterator;
    typedef MeshType::CellsContainer::Iterator         CellIterator;
    typedef CellType::PointIdIterator                  PointsInCellIterator;

  private:
    boost::filesystem::path _outputPath;
    std::ofstream           _outputMesh;
    std::ofstream           _outputMTL;

    itk::Mesh<::itk::Vector<double, 3>, 3>::Pointer _mesh;
    std::unordered_map<unsigned long, cv::Vec2d> _textCoords;
    cv::Mat _texture;

    int _writeHeader();
    int _writeVertices();
    int _writeTextureCoordinates();
    int _writeFaces();
  };


} // namespace io


} //namespace volcart

#endif //VC_IO_OBJEXPORT_H