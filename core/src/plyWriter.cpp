//
// Created by Media Team on 10/30/15.
//

#include "core/io/plyWriter.h"

namespace fs = boost::filesystem;

namespace volcart
{
namespace io
{

// Constructors
plyWriter::plyWriter(fs::path outputPath, ITKMesh::Pointer mesh)
{
    _outputPath = outputPath;
    _mesh = mesh;
};

plyWriter::plyWriter(
    fs::path outputPath, ITKMesh::Pointer mesh, volcart::Texture texture)
{
    _outputPath = outputPath;
    _mesh = mesh;
    _texture = texture;
};

// Make sure that all required parameters have been set and are okay
bool plyWriter::validate()
{

    // Make sure the output path has a file extension for the OBJ
    bool hasExt =
        (_outputPath.extension() == ".PLY" ||
         _outputPath.extension() == ".ply");
    // Make sure the output directory exists
    bool pathExists =
        fs::is_directory(fs::canonical(_outputPath.parent_path()));
    // Check that the mesh exists and has points
    bool meshHasPoints = (_mesh.IsNotNull() && _mesh->GetNumberOfPoints() != 0);

    return (hasExt && pathExists && meshHasPoints);
};

///// Output Methods /////
// Write everything (OBJ, MTL, and PNG) to disk
int plyWriter::write()
{
    if (!validate()) {
        return EXIT_FAILURE;
    }

    // Open the file stream
    _outputMesh.open(_outputPath.string());
    if (!_outputMesh.is_open()) {
        return EXIT_FAILURE;
    }

    // Capture the starting origin and set origin to what PLY reader needs
    Origin starting_origin = _texture.getUVMap().origin();
    _texture.getUVMap().origin(VC_ORIGIN_TOP_LEFT);

    _writeHeader();
    _writeVertices();
    _writeFaces();

    _outputMesh.close();

    // Restore the starting origin
    _texture.getUVMap().origin(starting_origin);

    return EXIT_SUCCESS;
};

// Write our custom header
int plyWriter::_writeHeader()
{
    if (!_outputMesh.is_open())
        return EXIT_FAILURE;

    _outputMesh << "ply" << std::endl;
    _outputMesh << "format ascii 1.0" << std::endl;
    _outputMesh << "comment VC PLY Exporter v1.0" << std::endl;

    // Vertex Info for Header
    _outputMesh << "element vertex " << _mesh->GetNumberOfPoints() << std::endl;
    _outputMesh << "property float x" << std::endl;
    _outputMesh << "property float y" << std::endl;
    _outputMesh << "property float z" << std::endl;
    _outputMesh << "property float nx" << std::endl;
    _outputMesh << "property float ny" << std::endl;
    _outputMesh << "property float nz" << std::endl;

    // Color info for vertices
    if (_texture.hasImages() && _texture.hasMap()) {
        _outputMesh << "property uchar red" << std::endl;
        _outputMesh << "property uchar green" << std::endl;
        _outputMesh << "property uchar blue" << std::endl;
    }

    // Face Info for Header
    if (_mesh->GetNumberOfCells() != 0) {
        _outputMesh << "element face " << _mesh->GetNumberOfCells()
                    << std::endl;
        _outputMesh << "property list uchar int vertex_indices" << std::endl;
    }

    // End header
    _outputMesh << "end_header" << std::endl;

    return EXIT_SUCCESS;
};

// Write the vertex information: 'v x y z'
//                               'vn nx ny nz'
int plyWriter::_writeVertices()
{
    if (!_outputMesh.is_open() || _mesh->GetNumberOfPoints() == 0)
        return EXIT_FAILURE;
    std::cerr << "Writing vertices..." << std::endl;

    // Iterate over all of the points
    for (auto point = _mesh->GetPoints()->Begin();
         point != _mesh->GetPoints()->End(); ++point) {

        // Get the point's normal
        ITKPixel normal;
        _mesh->GetPointData(point.Index(), &normal);

        // Write the point position components and its normal components.
        _outputMesh << point.Value()[0] << " " << point.Value()[1] << " "
                    << point.Value()[2] << " ";
        _outputMesh << normal[0] << " " << normal[1] << " " << normal[2];

        // If the texture has images and a uv map, write texture info
        if (_texture.hasImages() && _texture.hasMap()) {

            // Get the intensity for this point from the texture. If it doesn't
            // exist, set to 0.
            double intensity = _texture.intensity(point.Index());
            if (intensity != TEXTURE_NO_VALUE)
                intensity =
                    cvRound(intensity * 255.0 / 65535.0);  // map 16bit to 8bit
            else
                intensity = 0;

            _outputMesh << " " << (int)intensity << " " << (int)intensity << " "
                        << (int)intensity;
        }

        _outputMesh << std::endl;
    }

    return EXIT_SUCCESS;
};

// Write the face information: 'n#-of-verts v1 v1 ... vn'
int plyWriter::_writeFaces()
{
    if (!_outputMesh.is_open() || _mesh->GetNumberOfCells() == 0)
        return EXIT_FAILURE;
    std::cerr << "Writing faces..." << std::endl;

    // Iterate over the faces of the mesh
    ITKPointInCellIterator point;
    for (ITKCellIterator cell = _mesh->GetCells()->Begin();
         cell != _mesh->GetCells()->End(); ++cell) {
        _outputMesh << cell->Value()->GetNumberOfPoints();

        // Iterate over the points of this face and write the point IDs
        for (point = cell.Value()->PointIdsBegin();
             point != cell.Value()->PointIdsEnd(); ++point) {
            _outputMesh << " " << *point;
        }
        _outputMesh << std::endl;
    }

    return EXIT_SUCCESS;
};
}  // io
}  // volcart
