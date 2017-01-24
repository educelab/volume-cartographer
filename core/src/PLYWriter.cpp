//
// Created by Media Team on 10/30/15.
//

#include "core/io/PLYWriter.hpp"

namespace fs = boost::filesystem;
using namespace volcart::io;

// Make sure that all required parameters have been set and are okay
bool PLYWriter::validate()
{
    bool hasExt =
        outputPath_.extension() == ".PLY" || outputPath_.extension() == ".ply";
    bool pathExists =
        fs::is_directory(fs::canonical(outputPath_.parent_path()));
    bool meshHasPoints = mesh_.IsNotNull() && mesh_->GetNumberOfPoints() != 0;

    return hasExt && pathExists && meshHasPoints;
}

///// Output Methods /////
// Write everything (OBJ, MTL, and PNG) to disk
int PLYWriter::write()
{
    if (!validate()) {
        return EXIT_FAILURE;
    }

    // Open the file stream
    outputMesh_.open(outputPath_.string());
    if (!outputMesh_.is_open()) {
        return EXIT_FAILURE;
    }

    // Capture the starting origin and set origin to what PLY reader needs
    auto startingOrigin = texture_.uvMap().origin();
    texture_.uvMap().origin(VC_ORIGIN_TOP_LEFT);

    write_header_();
    write_vertices_();
    write_faces_();

    outputMesh_.close();

    // Restore the starting origin
    texture_.uvMap().origin(startingOrigin);

    return EXIT_SUCCESS;
}

// Write our custom header
int PLYWriter::write_header_()
{
    if (!outputMesh_.is_open()) {
        return EXIT_FAILURE;
    }

    outputMesh_ << "ply" << std::endl;
    outputMesh_ << "format ascii 1.0" << std::endl;
    outputMesh_ << "comment VC PLY Exporter v1.0" << std::endl;

    // Vertex Info for Header
    outputMesh_ << "element vertex " << mesh_->GetNumberOfPoints() << std::endl;
    outputMesh_ << "property float x" << std::endl;
    outputMesh_ << "property float y" << std::endl;
    outputMesh_ << "property float z" << std::endl;
    outputMesh_ << "property float nx" << std::endl;
    outputMesh_ << "property float ny" << std::endl;
    outputMesh_ << "property float nz" << std::endl;

    // Color info for vertices
    if (texture_.hasImages() && texture_.hasMap()) {
        outputMesh_ << "property uchar red" << std::endl;
        outputMesh_ << "property uchar green" << std::endl;
        outputMesh_ << "property uchar blue" << std::endl;
    }

    // Face Info for Header
    if (mesh_->GetNumberOfCells() != 0) {
        outputMesh_ << "element face " << mesh_->GetNumberOfCells()
                    << std::endl;
        outputMesh_ << "property list uchar int vertex_indices" << std::endl;
    }

    // End header
    outputMesh_ << "end_header" << std::endl;

    return EXIT_SUCCESS;
}

// Write the vertex information: 'v x y z'
//                               'vn nx ny nz'
int PLYWriter::write_vertices_()
{
    if (!outputMesh_.is_open() || mesh_->GetNumberOfPoints() == 0) {
        return EXIT_FAILURE;
    }
    std::cerr << "Writing vertices..." << std::endl;

    // Iterate over all of the points
    for (auto point = mesh_->GetPoints()->Begin();
         point != mesh_->GetPoints()->End(); ++point) {

        // Get the point's normal
        ITKPixel normal;
        mesh_->GetPointData(point.Index(), &normal);

        // Write the point position components and its normal components.
        outputMesh_ << point.Value()[0] << " " << point.Value()[1] << " "
                    << point.Value()[2] << " ";
        outputMesh_ << normal[0] << " " << normal[1] << " " << normal[2];

        // If the texture has images and a uv map, write texture info
        if (texture_.hasImages() && texture_.hasMap()) {

            // Get the intensity for this point from the texture. If it doesn't
            // exist, set to 0.
            double intensity = texture_.intensity(point.Index());
            if (intensity != TEXTURE_NO_VALUE) {
                // map 16bit to 8bit
                intensity = cvRound(intensity * 255.0 / 65535.0);
            } else {
                intensity = 0;
            }

            auto intIntensity = static_cast<int>(intensity);
            outputMesh_ << " " << intIntensity << " " << intIntensity << " "
                        << intIntensity;
        }

        outputMesh_ << std::endl;
    }

    return EXIT_SUCCESS;
}

// Write the face information: 'n#-of-verts v1 v1 ... vn'
int PLYWriter::write_faces_()
{
    if (!outputMesh_.is_open() || mesh_->GetNumberOfCells() == 0) {
        return EXIT_FAILURE;
    }
    std::cerr << "Writing faces..." << std::endl;

    // Iterate over the faces of the mesh
    ITKPointInCellIterator point;
    for (auto cell = mesh_->GetCells()->Begin();
         cell != mesh_->GetCells()->End(); ++cell) {
        outputMesh_ << cell->Value()->GetNumberOfPoints();

        // Iterate over the points of this face and write the point IDs
        for (point = cell.Value()->PointIdsBegin();
             point != cell.Value()->PointIdsEnd(); ++point) {
            outputMesh_ << " " << *point;
        }
        outputMesh_ << std::endl;
    }

    return EXIT_SUCCESS;
}
