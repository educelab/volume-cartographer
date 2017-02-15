//
// Created by Media Team on 6/24/15.
//
#include <string>

#include <opencv2/imgcodecs.hpp>

#include "vc/core/io/OBJWriter.hpp"

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

    // Setup material properties
    // Ka - Ambient light color
    // Kd - Diffuse light color
    // Ks - Specular light color
    // illum - Illumination mode
    // d - Dissolve. 1.0 == opaque
    std::cerr << "Writing MTL..." << std::endl;
    outputMTL_ << "newmtl default" << std::endl;
    outputMTL_ << "Ka 1.0 1.0 1.0" << std::endl;
    outputMTL_ << "Kd 1.0 1.0 1.0" << std::endl;
    outputMTL_ << "Ks 0.0 0.0 0.0" << std::endl;
    outputMTL_ << "illum 2" << std::endl;
    outputMTL_ << "d 1.0" << std::endl;

    // Path to the texture file, relative to the MTL file
    if (!texture_.empty()) {
        outputMTL_ << "map_Kd " << outputPath_.stem().string() + ".png"
                   << std::endl;
    }

    outputMTL_.close();
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

// Write the vertex information:
// Vertex: 'v x y z'
// Vertex normal: 'vn nx ny nz'
int OBJWriter::write_vertices_()
{
    if (!outputMesh_.is_open() || mesh_->GetNumberOfPoints() == 0) {
        return EXIT_FAILURE;
    }
    std::cerr << "Writing vertices..." << std::endl;

    outputMesh_ << "# Vertices: " << mesh_->GetNumberOfPoints() << std::endl;

    // Iterate over all of the points
    uint32_t vIndex = 1;
    uint32_t vnIndex = 1;
    for (auto pt = mesh_->GetPoints()->Begin(); pt != mesh_->GetPoints()->End();
         pt++) {
        // Make a new point link for this point
        cv::Vec3i pointLink(vIndex, UNSET_VALUE, UNSET_VALUE);

        // Write the point position components
        outputMesh_ << "v " << pt.Value()[0] << " " << pt.Value()[1] << " "
                    << pt.Value()[2] << std::endl;

        // Write the point normal information
        ITKPixel normal;
        if (mesh_->GetPointData(pt.Index(), &normal)) {
            outputMesh_ << "vn " << normal[0] << " " << normal[1] << " "
                        << normal[2] << std::endl;
            pointLink[2] = vnIndex++;
        }

        // Add this vertex to the point links
        pointLinks_.insert({pt.Index(), pointLink});

        ++vIndex;
    }

    return EXIT_SUCCESS;
}

// Write the UV coordinates that will be attached to points: 'vt u v'
int OBJWriter::write_texture_coordinates_()
{
    if (!outputMesh_.is_open() || textCoords_.empty()) {
        return EXIT_FAILURE;
    }
    std::cerr << "Writing texture coordinates..." << std::endl;

    // Ensure coordinates are relative to bottom left
    auto startingOrigin = textCoords_.origin();
    textCoords_.origin(VC_ORIGIN_BOTTOM_LEFT);

    // Write mtl path, relative to OBJ
    auto mtlpath = outputPath_.stem();
    mtlpath.replace_extension("mtl");
    outputMesh_ << "# Texture information" << std::endl;
    outputMesh_ << "mtllib " << mtlpath.string() << std::endl;
    outputMesh_ << "usemtl default" << std::endl;

    // Iterate over all of the saved coordinates in our coordinate map
    uint32_t vtIndex = 1;
    for (uint32_t pId = 0; pId < textCoords_.size(); ++pId) {
        cv::Vec2d uv = textCoords_.get(pId);
        outputMesh_ << "vt " << uv[0] << " " << uv[1] << std::endl;

        // Find this UV map's point in _point_links and set its vt value to our
        // current position in the vt list
        pointLinks_.find(pId)->second[1] = vtIndex;

        ++vtIndex;
    }

    // Restore the starting origin
    textCoords_.origin(startingOrigin);
    return EXIT_SUCCESS;
}

// Write the face information: 'f v/vt/vn'
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

            cv::Vec3i pointLink = pointLinks_.find(*point)->second;

            outputMesh_ << pointLink[0];

            // Write the vtIndex
            if (pointLink[1] != UNSET_VALUE) {
                outputMesh_ << "/" << pointLink[1];
            }

            // Write the vnIndex
            if (pointLink[2] != UNSET_VALUE) {
                // Write a buffer slash if there wasn't a vtIndex
                if (pointLink[1] == UNSET_VALUE) {
                    outputMesh_ << "/";
                }

                outputMesh_ << "/" << pointLink[2];
            }

            outputMesh_ << " ";
        }
        outputMesh_ << std::endl;
    }

    return EXIT_SUCCESS;
}
