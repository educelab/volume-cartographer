//
// Created by Media Team on 6/24/15.
//

#include "io_objExport.h"

namespace volcart {
    namespace io {

    ///// Constructors //////
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

    ///// Output Methods /////

    // Write everything (OBJ, MTL, and PNG) to disk
    int objWriter::write() {

        writeMesh();

        if ( !_textCoords.empty() ) {
            writeMTL();
            writeTexture();
        }

        return EXIT_SUCCESS;
    }

    // Write the OBJ file to disk
    int objWriter::writeMesh() {
        _outputMesh.open( _outputPath.string() ); // Open the file stream
        if(!_outputMesh.is_open()) return EXIT_FAILURE; // Return error if we can't open the file

        _writeHeader();
        _writeVertices();
        if ( !_textCoords.empty() ) _writeTextureCoordinates(); // Only write texture information if we have a UV map
        _writeFaces();

        _outputMesh.close(); // Close the file stream

        return EXIT_SUCCESS;
    };

    // Write the MTL file to disk
    // See http://paulbourke.net/dataformats/mtl/ for more options
    int objWriter::writeMTL() {
        _outputMTL.open( _outputPath.stem().string() + ".mtl" ); // Open the file stream
        if(!_outputMTL.is_open()) return EXIT_FAILURE; // Return error if we can't open the file

        std::cerr << "Writing MTL..." << std::endl;

        _outputMTL << "newmtl default" << std::endl;

        _outputMTL << "Ka 1.0 1.0 1.0" << std::endl;   // Ambient light color
        _outputMTL << "Kd 1.0 1.0 1.0" << std::endl;   // Diffuse light color
        _outputMTL << "Ks 0.0 0.0 0.0" << std::endl;   // Specular light color
        _outputMTL << "illum 2" << std::endl;          // Illumination mode
        _outputMTL << "d 1.0" << std::endl;            // Dissolve. 1.0 == opaque

        _outputMTL << "map_Kd " << _outputPath.stem().string() + ".png" << std::endl; // Path to the texture file, relative to the MTL file

        _outputMTL.close(); // Close the file stream
        return EXIT_SUCCESS;
    };

    // Write the PNG texture file to disk
    int objWriter::writeTexture() {
        if ( _texture.empty() ) return EXIT_FAILURE;

        std::cerr << "Writing texture image..." << std::endl;
        imwrite( _outputPath.stem().string() + ".png", _texture );
        return EXIT_SUCCESS;
    };

    // Write our custom header
    int objWriter::_writeHeader() {
        if(!_outputMesh.is_open()) return EXIT_FAILURE;

        _outputMesh << "# VolCart OBJ File" << std::endl;
        _outputMesh << "# VC OBJ Exporter v1.0" << std::endl;

        return EXIT_SUCCESS;
    };

    // Write the vertex information: 'v x y z'
    //                               'vn nx ny nz'
    int objWriter::_writeVertices() {
        if(!_outputMesh.is_open()) return EXIT_FAILURE;
        std::cerr << "Writing vertices..." << std::endl;

        _outputMesh << "# Vertices: " << _mesh->GetPoints()->Size() << std::endl;

        // Iterate over all of the points
        PointsInMeshIterator point = _mesh->GetPoints()->Begin();
        while ( point != _mesh->GetPoints()->End() ) {

            // Get the point's normal
            MeshType::PixelType normal;
            _mesh->GetPointData(point.Index(), &normal);

            // Write the point position components and its normal components.
            _outputMesh << "v "  << point.Value()[0] << " " << point.Value()[1] << " " << point.Value()[2] << std::endl;
            _outputMesh << "vn " << normal[0] << " " << normal[1] << " " << normal[2] << std::endl;

            ++point;
        }

        return EXIT_SUCCESS;
    };

    // Write the UV coordinates that will be attached to points: 'vt u v'
    int objWriter::_writeTextureCoordinates() {
        if(!_outputMesh.is_open()) return EXIT_FAILURE;
        std::cerr << "Writing texture coordinates..." << std::endl;

        _outputMesh << "# Texture information" << std::endl;
        _outputMesh << "mtllib " << _outputPath.stem().string() << ".mtl" << std::endl;  // The path of the MTL file, relative to the obj
        _outputMesh << "usemtl default" << std::endl;  // Use the material named 'default' in the MTL file

        // Iterate over all of the saved coordinates in our coordinate map
        auto coordinate = _textCoords.begin(); // The map iterator
        while ( coordinate != _textCoords.end() ) {
            // Map iterators return pairs: {first = key, second = cv::Vec2d}
            // [u, v] == [ second[0], second[1] ]
            _outputMesh << "vt " << coordinate->second[0] << " " << coordinate->second[1] << std::endl;
            ++coordinate;
        }

        return EXIT_SUCCESS;
    }

    // Write the face information: 'f v/vt/vn'
    // Note: This method currently assumes that *every* point in the mesh has an associated normal and texture map
    // This will definitely not always be the case and should be fixed. - SP
    int objWriter::_writeFaces() {
        if(!_outputMesh.is_open()) return EXIT_FAILURE;
        std::cerr << "Writing faces..." << std::endl;

        _outputMesh << "# Faces: " << _mesh->GetCells()->size() << std::endl;

        // Iterate over the faces of the mesh
        CellIterator cell = _mesh->GetCells()->Begin();
        PointsInCellIterator point;

        while ( cell != _mesh->GetCells()->End() ) {
            _outputMesh << "f "; // Starts a new face line

            // Iterate over the points of this face
            point = cell.Value()->PointIdsBegin();
            while ( point != cell.Value()->PointIdsEnd() ) {
                unsigned long pointIndex = *point + 1; // OBJ elements are indexed from 1, not 0
                std::string textureIndex = "";

                // Set the texture index if we have texture coordinates
                // To-Do: Set the texture index only if this point has a texture coordinate
                if ( !_textCoords.empty() )
                   textureIndex = std::to_string(pointIndex);

                _outputMesh << pointIndex << "/" << textureIndex << "/" << pointIndex << " ";

                ++point;
            }
            _outputMesh << std::endl;
            ++cell;
        }

        return EXIT_SUCCESS;
    }

    } // namespace io
} //namespace volcart