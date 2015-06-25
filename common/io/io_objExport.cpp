//
// Created by Media Team on 6/24/15.
//

#include "io_objExport.h"

namespace volcart {
    namespace io {

    objWriter::objWriter( std::string outputPath, itk::Mesh<::itk::Vector<double, 3>, 3>::Pointer mesh ) {
        _outputPath = outputPath;
        _mesh = mesh;
    };

    objWriter::objWriter( std::string outputPath, itk::Mesh<::itk::Vector<double, 3>, 3>::Pointer mesh, std::unordered_map<unsigned long, cv::Vec2d> uvMap, cv::Mat uvImg ) {
        _outputPath = outputPath;
        _mesh = mesh;
        _textCoords = uvMap;
        _texture = uvImg;
    };

    int objWriter::write() {

        writeMesh();

        if ( !_textCoords.empty() ) {
            writeMTL();
            writeTexture();
        }

        return EXIT_SUCCESS;
    }

    int objWriter::writeMesh() {
        _outputMesh.open( _outputPath.string() );
        if(!_outputMesh.is_open()) return EXIT_FAILURE;

        _writeHeader();
        _writeVertices();
        if ( !_textCoords.empty() ) _writeTextureCoordinates();
        _writeFaces();

        _outputMesh.close();

        return EXIT_SUCCESS;
    };

    int objWriter::writeMTL() {
        _outputMTL.open( _outputPath.stem().string() + ".mtl" );
        if(!_outputMTL.is_open()) return EXIT_FAILURE;

        std::cerr << "Writing MTL..." << std::endl;

        _outputMTL << "newmtl default" << std::endl;

        _outputMTL << "Ka 0.5 0.5 0.5" << std::endl;
        _outputMTL << "Kd 0.5 0.5 0.5" << std::endl;
        _outputMTL << "Ks 0.5 0.5 0.5" << std::endl;
        _outputMTL << "Tf 0.5 0.5 0.5" << std::endl;
        _outputMTL << "illum 1" << std::endl;
        _outputMTL << "d 1" << std::endl;
        _outputMTL << "Ns 6" << std::endl;
        _outputMTL << "sharpness 60" << std::endl;
        _outputMTL << "Ni 1" << std::endl;

        _outputMTL << "map_Kd " << _outputPath.stem().string() + ".png" << std::endl;

        _outputMTL.close();
        return EXIT_SUCCESS;
    };

    int objWriter::writeTexture() {
        if ( _texture.empty() ) return EXIT_FAILURE;

        std::cerr << "Writing texture image..." << std::endl;
        imwrite( _outputPath.stem().string() + ".png", _texture );
        return EXIT_SUCCESS;
    };

    int objWriter::_writeHeader() {
        if(!_outputMesh.is_open()) return EXIT_FAILURE;

        _outputMesh << "# VolCart OBJ File" << std::endl;
        _outputMesh << "# VC OBJ Exporter v1.0" << std::endl;
        _outputMesh << std::endl;

        return EXIT_SUCCESS;
    };

    int objWriter::_writeVertices() {
        if(!_outputMesh.is_open()) return EXIT_FAILURE;
        std::cerr << "Writing vertices..." << std::endl;

        _outputMesh << "# Vertices: " << _mesh->GetPoints()->Size() << std::endl;

        PointsInMeshIterator point = _mesh->GetPoints()->Begin();
        while ( point != _mesh->GetPoints()->End() ) {

            MeshType::PixelType normal;
            _mesh->GetPointData(point.Index(), &normal);

            _outputMesh << "v "  << point.Value()[0] << " " << point.Value()[1] << " " << point.Value()[2] << std::endl;
            _outputMesh << "vn " << normal[0] << " " << normal[1] << " " << normal[2] << std::endl;

            ++point;
        }

        _outputMesh << std::endl;
        return EXIT_SUCCESS;
    };

    int objWriter::_writeTextureCoordinates() {
        if(!_outputMesh.is_open()) return EXIT_FAILURE;
        std::cerr << "Writing texture coordinates..." << std::endl;

        _outputMesh << "# Texture information" << std::endl;
        _outputMesh << "mtllib " << _outputPath.stem().string() << ".mtl" << std::endl;
        _outputMesh << "usemtl default" << std::endl;

        PointsInMeshIterator point = _mesh->GetPoints()->Begin();
        while ( point != _mesh->GetPoints()->End() ) {
            _outputMesh << "vt " << _textCoords.at(point.Index())[0] << " " << _textCoords.at(point.Index())[1] << std::endl;
            ++point;
        }

        _outputMesh << std::endl;
        return EXIT_SUCCESS;
    }

    int objWriter::_writeFaces() {
        if(!_outputMesh.is_open()) return EXIT_FAILURE;
        std::cerr << "Writing faces..." << std::endl;

        _outputMesh << "# Faces: " << _mesh->GetCells()->size() << std::endl;

        CellIterator cell = _mesh->GetCells()->Begin();
        PointsInCellIterator point;

        while ( cell != _mesh->GetCells()->End() ) {
            _outputMesh << "f ";

            point = cell.Value()->PointIdsBegin();
            while ( point != cell.Value()->PointIdsEnd() ) {

                unsigned long pointIndex = *point + 1; // OBJ elements are indexed from 1, not 0

                if ( _textCoords.empty() )
                    _outputMesh << pointIndex << "//" << pointIndex << " ";
                else
                    _outputMesh << pointIndex << "/" << pointIndex << "/" << pointIndex << " ";

                ++point;
            }
            _outputMesh << std::endl;
            ++cell;
        }

        _outputMesh << std::endl;
        return EXIT_SUCCESS;
    }

    } // namespace io
} //namespace volcart