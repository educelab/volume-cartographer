//
// Created by Media Team on 6/24/15.
//
#include <string>

#include <opencv2/imgcodecs.hpp>

#include "core/io/OBJWriter.hpp"

namespace fs = boost::filesystem;

static constexpr int UNSET_VALUE = -1;

using namespace volcart::io;

///// Access Functions /////
void OBJWriter::setRendering(const volcart::Rendering& rendering)
{
    mesh_ = rendering.getMesh();
    texture_ = rendering.getTexture().image(0);
    textCoords_ = rendering.getTexture().uvMap();
}

///// Validation /////
// Make sure that all required parameters have been set and are okay
bool OBJWriter::validate()
{
    // Make sure the output path has a file extension for the OBJ
    bool hasExt =
        (outputPath_.extension() == ".OBJ" ||
         outputPath_.extension() == ".obj");
    // Make sure the output directory exists
    bool pathExists =
        fs::is_directory(fs::canonical(outputPath_.parent_path()));
    // Check that the mesh exists and has points
    bool meshHasPoints = (mesh_.IsNotNull() && mesh_->GetNumberOfPoints() != 0);

    return (hasExt && pathExists && meshHasPoints);
}

///// Output Methods /////
// Write everything (OBJ, MTL, and PNG) to disk
int OBJWriter::write()
{
    if (!validate()) {
        return EXIT_FAILURE;
    }

    // Write the OBJ
    writeOBJ();

    // Write texture stuff if we have a UV coordinate map
    if (!textCoords_.empty()) {
        writeMTL();
        writeTexture();
    }

    return EXIT_SUCCESS;
}

// Write the OBJ file to disk
int OBJWriter::writeOBJ()
{
    outputMesh_.open(outputPath_.string());
    if (!outputMesh_.is_open()) {
        return EXIT_FAILURE;
    }

    write_header_();
    write_vertices_();

    // Only write texture information if we have a UV map
    if (!textCoords_.empty()) {
        write_texture_coordinates_();
    }

    write_faces_();
    outputMesh_.close();

    return EXIT_SUCCESS;
}

// Write the MTL file to disk
// See http://paulbourke.net/dataformats/mtl/ for more options
int OBJWriter::writeMTL()
{
    fs::path p = outputPath_;
    p.replace_extension("mtl");
    outputMTL_.open(p.string());
    if (!outputMTL_.is_open()) {
        return EXIT_FAILURE;
    }

    std::cerr << "Writing MTL..." << std::endl;
    outputMTL_ << "newmtl default" << std::endl;
    outputMTL_ << "Ka 1.0 1.0 1.0" << std::endl;  // Ambient light color
    outputMTL_ << "Kd 1.0 1.0 1.0" << std::endl;  // Diffuse light color
    outputMTL_ << "Ks 0.0 0.0 0.0" << std::endl;  // Specular light color
    outputMTL_ << "illum 2" << std::endl;         // Illumination mode
    outputMTL_ << "d 1.0" << std::endl;           // Dissolve. 1.0 == opaque

    // Path to the texture file, relative to the MTL file
    if (!texture_.empty()) {
        outputMTL_ << "map_Kd " << outputPath_.stem().string() + ".png"
                   << std::endl;
    }

    outputMTL_.close();  // Close the file stream
    return EXIT_SUCCESS;
}

// Write the PNG texture file to disk
int OBJWriter::writeTexture()
{
    if (texture_.empty()) {
        return EXIT_FAILURE;
    }

    std::cerr << "Writing texture image..." << std::endl;
    fs::path p = outputPath_;
    p.replace_extension("png");
    cv::imwrite(p.string(), texture_);
    return EXIT_SUCCESS;
}

// Write our custom header
int OBJWriter::write_header_()
{
    if (!outputMesh_.is_open()) {
        return EXIT_FAILURE;
    }

    outputMesh_ << "# VolCart OBJ File" << std::endl;
    outputMesh_ << "# VC OBJ Exporter v1.0" << std::endl;
    return EXIT_SUCCESS;
}

// Write the vertex information: 'v x y z'
//                               'vn nx ny nz'
int OBJWriter::write_vertices_()
{
    if (!outputMesh_.is_open() || mesh_->GetNumberOfPoints() == 0) {
        return EXIT_FAILURE;
    }
    std::cerr << "Writing vertices..." << std::endl;

    outputMesh_ << "# Vertices: " << mesh_->GetNumberOfPoints() << std::endl;

    // Iterate over all of the points
    auto point = mesh_->GetPoints()->Begin();
    double vIndex = 1;
    while (point != mesh_->GetPoints()->End()) {

        // Get the point's normal
        ITKPixel normal;
        mesh_->GetPointData(point.Index(), &normal);

        // Write the point position components and its normal components.
        outputMesh_ << "v " << point.Value()[0] << " " << point.Value()[1]
                    << " " << point.Value()[2] << std::endl;
        outputMesh_ << "vn " << normal[0] << " " << normal[1] << " "
                    << normal[2] << std::endl;

        // Make a new point link for this point
        cv::Vec3d pointLink(vIndex, UNSET_VALUE, vIndex);
        pointLinks_.insert({point.Index(), pointLink});

        ++vIndex;
        ++point;
    }

    return EXIT_SUCCESS;
}

// Write the UV coordinates that will be attached to points: 'vt u v'
// To-Do: #179
int OBJWriter::write_texture_coordinates_()
{
    if (!outputMesh_.is_open() || textCoords_.empty()) {
        return EXIT_FAILURE;
    }
    std::cerr << "Writing texture coordinates..." << std::endl;

    // Ensure coordinates are relative to bottom left
    auto startingOrigin = textCoords_.origin();
    textCoords_.origin(VC_ORIGIN_BOTTOM_LEFT);

    outputMesh_ << "# Texture information" << std::endl;
    outputMesh_ << "mtllib " << outputPath_.stem().string() << ".mtl"
                << std::endl;  // The path of the MTL file, relative to the obj
    outputMesh_
        << "usemtl default"
        << std::endl;  // Use the material named 'default' in the MTL file

    // Iterate over all of the saved coordinates in our coordinate map
    double vtIndex = 1;
    for (size_t pId = 0; pId < textCoords_.size(); ++pId) {
        cv::Vec2d uv = textCoords_.get(pId);
        outputMesh_ << "vt " << uv[0] << " " << uv[1] << std::endl;

        // Find this UV map's point in _point_links and set its vt value to our
        // current position in the vt list
        pointLinks_.find(pId)->second[1] = vtIndex;

        ++vtIndex;
    }

    textCoords_.origin(startingOrigin);  // Restore the starting origin
    return EXIT_SUCCESS;
}

// Write the face information: 'f v/vt/vn'
// Note: This method currently assumes that *every* point in the mesh has an
// associated normal and texture map
// This will definitely not always be the case and should be fixed. - SP
int OBJWriter::write_faces_()
{
    if (!outputMesh_.is_open() || mesh_->GetNumberOfCells() == 0) {
        return EXIT_FAILURE;
    }
    std::cerr << "Writing faces..." << std::endl;

    outputMesh_ << "# Faces: " << mesh_->GetNumberOfCells() << std::endl;

    // Iterate over the faces of the mesh
    ITKPointInCellIterator point;

    for (auto cell = mesh_->GetCells()->Begin();
         cell != mesh_->GetCells()->End(); ++cell) {
        // Starts a new face line
        outputMesh_ << "f ";

        // Iterate over the points of this face
        for (point = cell.Value()->PointIdsBegin();
             point != cell.Value()->PointIdsEnd(); ++point) {
            std::string vIndex, vtIndex, vnIndex;

            cv::Vec3d pointLink = pointLinks_.find(*point)->second;

            vIndex = std::to_string(pointLink[0]);
            if (pointLink[1] != UNSET_VALUE) {
                vtIndex = std::to_string(pointLink[1]);
            }
            vnIndex = std::to_string(pointLink[2]);

            outputMesh_ << vIndex << "/" << vtIndex << "/" << vnIndex << " ";
        }
        outputMesh_ << std::endl;
    }

    return EXIT_SUCCESS;
}
