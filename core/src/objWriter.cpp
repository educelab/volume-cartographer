//
// Created by Media Team on 6/24/15.
//

#include "core/io/objWriter.h"

namespace fs = boost::filesystem;

#define UNSET_VALUE -1

namespace volcart
{
namespace io
{

///// Constructors /////
objWriter::objWriter(){};

objWriter::objWriter(fs::path outputPath, ITKMesh::Pointer mesh)
{
    _outputPath = outputPath;
    _mesh = mesh;
};

objWriter::objWriter(
    fs::path outputPath,
    ITKMesh::Pointer mesh,
    volcart::UVMap uvMap,
    cv::Mat uvImg)
{
    _outputPath = outputPath;
    _mesh = mesh;
    _textCoords = uvMap;
    _texture = uvImg;
};

///// Access Functions /////
void objWriter::setRendering(volcart::Rendering rendering)
{
    _mesh = rendering.getMesh();
    _texture = rendering.getTexture().getImage(0);
    _textCoords = rendering.getTexture().uvMap();
}

///// Validation /////
// Make sure that all required parameters have been set and are okay
bool objWriter::validate()
{

    // Make sure the output path has a file extension for the OBJ
    bool hasExt =
        (_outputPath.extension() == ".OBJ" ||
         _outputPath.extension() == ".obj");
    // Make sure the output directory exists
    bool pathExists =
        fs::is_directory(fs::canonical(_outputPath.parent_path()));
    // Check that the mesh exists and has points
    bool meshHasPoints = (_mesh.IsNotNull() && _mesh->GetNumberOfPoints() != 0);

    return (hasExt && pathExists && meshHasPoints);
};

///// Output Methods /////
// Write everything (OBJ, MTL, and PNG) to disk
int objWriter::write()
{
    if (!validate())
        return EXIT_FAILURE;  // Must pass validation test

    // Write the OBJ
    writeOBJ();

    // Write texture stuff if we have a UV coordinate map
    if (!_textCoords.empty()) {
        writeMTL();
        writeTexture();
    }

    return EXIT_SUCCESS;
};

// Write the OBJ file to disk
int objWriter::writeOBJ()
{
    _outputMesh.open(_outputPath.string());  // Open the file stream
    if (!_outputMesh.is_open())
        return EXIT_FAILURE;  // Return error if we can't open the file

    _writeHeader();
    _writeVertices();
    if (!_textCoords.empty())
        _writeTextureCoordinates();  // Only write texture information if we
                                     // have a UV map
    _writeFaces();

    _outputMesh.close();  // Close the file stream

    return EXIT_SUCCESS;
};

// Write the MTL file to disk
// See http://paulbourke.net/dataformats/mtl/ for more options
int objWriter::writeMTL()
{
    fs::path p = _outputPath;
    p.replace_extension("mtl");
    _outputMTL.open(p.string());  // Open the file stream
    if (!_outputMTL.is_open())
        return EXIT_FAILURE;  // Return error if we can't open the file

    std::cerr << "Writing MTL..." << std::endl;

    _outputMTL << "newmtl default" << std::endl;

    _outputMTL << "Ka 1.0 1.0 1.0" << std::endl;  // Ambient light color
    _outputMTL << "Kd 1.0 1.0 1.0" << std::endl;  // Diffuse light color
    _outputMTL << "Ks 0.0 0.0 0.0" << std::endl;  // Specular light color
    _outputMTL << "illum 2" << std::endl;         // Illumination mode
    _outputMTL << "d 1.0" << std::endl;           // Dissolve. 1.0 == opaque

    if (!_texture.empty())
        _outputMTL
            << "map_Kd " << _outputPath.stem().string() + ".png"
            << std::endl;  // Path to the texture file, relative to the MTL file

    _outputMTL.close();  // Close the file stream
    return EXIT_SUCCESS;
};

// Write the PNG texture file to disk
int objWriter::writeTexture()
{
    if (_texture.empty())
        return EXIT_FAILURE;

    std::cerr << "Writing texture image..." << std::endl;
    fs::path p = _outputPath;
    p.replace_extension("png");
    imwrite(p.string(), _texture);
    return EXIT_SUCCESS;
};

// Write our custom header
int objWriter::_writeHeader()
{
    if (!_outputMesh.is_open())
        return EXIT_FAILURE;

    _outputMesh << "# VolCart OBJ File" << std::endl;
    _outputMesh << "# VC OBJ Exporter v1.0" << std::endl;

    return EXIT_SUCCESS;
};

// Write the vertex information: 'v x y z'
//                               'vn nx ny nz'
int objWriter::_writeVertices()
{
    if (!_outputMesh.is_open() || _mesh->GetNumberOfPoints() == 0)
        return EXIT_FAILURE;
    std::cerr << "Writing vertices..." << std::endl;

    _outputMesh << "# Vertices: " << _mesh->GetNumberOfPoints() << std::endl;

    // Iterate over all of the points
    ITKPointIterator point = _mesh->GetPoints()->Begin();
    double v_Index = 1;
    while (point != _mesh->GetPoints()->End()) {

        // Get the point's normal
        ITKPixel normal;
        _mesh->GetPointData(point.Index(), &normal);

        // Write the point position components and its normal components.
        _outputMesh << "v " << point.Value()[0] << " " << point.Value()[1]
                    << " " << point.Value()[2] << std::endl;
        _outputMesh << "vn " << normal[0] << " " << normal[1] << " "
                    << normal[2] << std::endl;

        // Make a new point link for this point
        cv::Vec3d point_link(v_Index, UNSET_VALUE, v_Index);
        _point_links.insert({point.Index(), point_link});

        ++v_Index;
        ++point;
    }

    return EXIT_SUCCESS;
};

// Write the UV coordinates that will be attached to points: 'vt u v'
// To-Do: Separate out the mtllib and mtl assignment
int objWriter::_writeTextureCoordinates()
{
    if (!_outputMesh.is_open() || _textCoords.empty())
        return EXIT_FAILURE;
    std::cerr << "Writing texture coordinates..." << std::endl;

    // Ensure coordinates are relative to bottom left
    Origin starting_origin =
        _textCoords.origin();  // Capture the starting origin
    _textCoords.origin(VC_ORIGIN_BOTTOM_LEFT);

    _outputMesh << "# Texture information" << std::endl;
    _outputMesh << "mtllib " << _outputPath.stem().string() << ".mtl"
                << std::endl;  // The path of the MTL file, relative to the obj
    _outputMesh
        << "usemtl default"
        << std::endl;  // Use the material named 'default' in the MTL file

    // Iterate over all of the saved coordinates in our coordinate map
    double vt_Index = 1;
    for (double p_id = 0; p_id < _textCoords.size(); ++p_id) {
        cv::Vec2d uv = _textCoords.get(p_id);
        _outputMesh << "vt " << uv[0] << " " << uv[1] << std::endl;

        // Find this UV map's point in _point_links and set its vt value to our
        // current position in the vt list
        _point_links.find(p_id)->second[1] = vt_Index;

        ++vt_Index;
    }

    _textCoords.origin(starting_origin);  // Restore the starting origin
    return EXIT_SUCCESS;
};

// Write the face information: 'f v/vt/vn'
// Note: This method currently assumes that *every* point in the mesh has an
// associated normal and texture map
// This will definitely not always be the case and should be fixed. - SP
int objWriter::_writeFaces()
{
    if (!_outputMesh.is_open() || _mesh->GetNumberOfCells() == 0)
        return EXIT_FAILURE;
    std::cerr << "Writing faces..." << std::endl;

    _outputMesh << "# Faces: " << _mesh->GetNumberOfCells() << std::endl;

    // Iterate over the faces of the mesh
    ITKPointInCellIterator point;

    for (ITKCellIterator cell = _mesh->GetCells()->Begin();
         cell != _mesh->GetCells()->End(); ++cell) {
        _outputMesh << "f ";  // Starts a new face line

        // Iterate over the points of this face
        for (point = cell.Value()->PointIdsBegin();
             point != cell.Value()->PointIdsEnd(); ++point) {
            std::string v_Index = "";
            std::string vt_Index = "";
            std::string vn_Index = "";

            cv::Vec3d point_link = _point_links.find(*point)->second;

            v_Index = boost::lexical_cast<std::string>(point_link[0]);
            if (point_link[1] != UNSET_VALUE)
                vt_Index = boost::lexical_cast<std::string>(point_link[1]);
            vn_Index = boost::lexical_cast<std::string>(point_link[2]);

            _outputMesh << v_Index << "/" << vt_Index << "/" << vn_Index << " ";
        }
        _outputMesh << std::endl;
    }

    return EXIT_SUCCESS;
};

}  // namespace io
}  // namespace volcart
