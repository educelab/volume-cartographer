#include "vc/core/io/OBJWriter.hpp"

#include <string>

#include "vc/core/io/ImageIO.hpp"
#include "vc/core/types/Exceptions.hpp"
#include "vc/core/util/Logging.hpp"

static constexpr int UNSET_VALUE = -1;

using namespace volcart;
using namespace volcart::io;

namespace fs = volcart::filesystem;

OBJWriter::OBJWriter(fs::path outputPath, ITKMesh::Pointer mesh)
    : outputPath_{std::move(outputPath)}, mesh_{std::move(mesh)}
{
}

OBJWriter::OBJWriter(
    fs::path outputPath,
    ITKMesh::Pointer mesh,
    UVMap::Pointer uvMap,
    cv::Mat uvImg)
    : outputPath_{std::move(outputPath)}
    , mesh_{std::move(mesh)}
    , uvMap_{std::move(uvMap)}
    , texture_{std::move(uvImg)}
{
}

void OBJWriter::setPath(const filesystem::path& path) { outputPath_ = path; }
void OBJWriter::setMesh(ITKMesh::Pointer mesh) { mesh_ = std::move(mesh); }
void OBJWriter::setUVMap(UVMap::Pointer uvMap) { uvMap_ = std::move(uvMap); }
void OBJWriter::setTexture(cv::Mat uvImg) { texture_ = std::move(uvImg); }
void OBJWriter::setTextureFormat(std::string fmt)
{
    textureFmt_ = std::move(fmt);
}

///// Output Methods /////
// Write everything (OBJ, MTL, and PNG) to disk
auto OBJWriter::write() -> int
{
    if (not mesh_ or mesh_->GetNumberOfPoints() == 0) {
        throw volcart::IOException("Mesh is empty or null");
    }

    // Write the OBJ
    write_obj_();

    // Write texture stuff if we have a UV coordinate map
    if (uvMap_ and not uvMap_->empty()) {
        write_mtl_();
        write_texture_();
    }

    return EXIT_SUCCESS;
}

// Write the OBJ file to disk
auto OBJWriter::write_obj_() -> int
{
    Logger()->debug("Writing OBJ mesh: {}", outputPath_.string());
    outputMesh_.open(outputPath_.string());
    if (!outputMesh_.is_open()) {
        Logger()->error("Failed to write OBJ mesh");
        return EXIT_FAILURE;
    }

    write_header_();
    write_vertices_();

    // Only write texture information if we have a UV map
    if (uvMap_ and not uvMap_->empty()) {
        write_texture_coordinates_();
    }

    write_faces_();
    outputMesh_.close();

    return EXIT_SUCCESS;
}

// Write the MTL file to disk
// See http://paulbourke.net/dataformats/mtl/ for more options
auto OBJWriter::write_mtl_() -> int
{
    fs::path p = outputPath_;
    p.replace_extension("mtl");
    outputMTL_.open(p.string());
    if (not outputMTL_.is_open()) {
        return EXIT_FAILURE;
    }

    // Setup material properties
    // Ka - Ambient light color
    // Kd - Diffuse light color
    // Ks - Specular light color
    // illum - Illumination mode
    // d - Dissolve. 1.0 == opaque
    Logger()->debug("Writing MTL...");
    outputMTL_ << "newmtl default\n";
    outputMTL_ << "Ka 1.0 1.0 1.0\n";
    outputMTL_ << "Kd 1.0 1.0 1.0\n";
    outputMTL_ << "Ks 0.0 0.0 0.0\n";
    outputMTL_ << "illum 2\n";
    outputMTL_ << "d 1.0\n";

    // Path to the texture file, relative to the MTL file
    if (!texture_.empty()) {
        auto textureFile = outputPath_;
        textureFile.replace_extension(textureFmt_);
        outputMTL_ << "map_Kd " << textureFile.filename().string() << "\n";
    }

    outputMTL_.close();
    return EXIT_SUCCESS;
}

// Write the PNG texture file to disk
auto OBJWriter::write_texture_() -> int
{
    if (texture_.empty()) {
        return EXIT_FAILURE;
    }

    Logger()->debug("Writing texture image...");
    auto p = outputPath_;
    p.replace_extension(textureFmt_);
    WriteImage(p, texture_);
    return EXIT_SUCCESS;
}

// Write our custom header
auto OBJWriter::write_header_() -> int
{
    if (!outputMesh_.is_open()) {
        return EXIT_FAILURE;
    }

    outputMesh_ << "# VolCart OBJ File\n";
    outputMesh_ << "# VC OBJ Exporter v1.0\n";
    return EXIT_SUCCESS;
}

// Write the vertex information:
// Vertex: 'v x y z'
// Vertex normal: 'vn nx ny nz'
auto OBJWriter::write_vertices_() -> int
{
    if (!outputMesh_.is_open() || mesh_->GetNumberOfPoints() == 0) {
        return EXIT_FAILURE;
    }
    Logger()->debug("Writing vertices...");

    outputMesh_ << "# Vertices: " << mesh_->GetNumberOfPoints() << "\n";

    // Iterate over all of the points
    uint32_t vIndex = 1;
    uint32_t vnIndex = 1;
    for (auto pt = mesh_->GetPoints()->Begin(); pt != mesh_->GetPoints()->End();
         pt++) {
        // Make a new point link for this point
        cv::Vec3i pointLink(vIndex, UNSET_VALUE, UNSET_VALUE);

        // Write the point position components
        outputMesh_ << "v " << pt.Value()[0] << " " << pt.Value()[1] << " "
                    << pt.Value()[2] << "\n";

        // Write the point normal information
        ITKPixel normal;
        if (mesh_->GetPointData(pt.Index(), &normal)) {
            outputMesh_ << "vn " << normal[0] << " " << normal[1] << " "
                        << normal[2] << "\n";
            pointLink[2] = vnIndex++;
        }

        // Add this vertex to the point links
        pointLinks_.insert({pt.Index(), pointLink});

        ++vIndex;
    }

    return EXIT_SUCCESS;
}

// Write the UV coordinates that will be attached to points: 'vt u v'
auto OBJWriter::write_texture_coordinates_() -> int
{
    if (not outputMesh_.is_open() or uvMap_->empty()) {
        return EXIT_FAILURE;
    }
    Logger()->debug("Writing texture coordinates...");

    // Ensure coordinates are relative to bottom left
    auto startingOrigin = uvMap_->origin();
    uvMap_->setOrigin(UVMap::Origin::BottomLeft);

    // Write mtl path, relative to OBJ
    auto mtlpath = outputPath_.stem();
    mtlpath.replace_extension("mtl");
    outputMesh_ << "# Texture information\n";
    outputMesh_ << "mtllib " << mtlpath.string() << "\n";
    outputMesh_ << "usemtl default\n";

    // Iterate over all of the saved coordinates in our coordinate map
    uint32_t vtIndex = 1;
    for (uint32_t pId = 0; pId < uvMap_->size(); ++pId) {
        cv::Vec2d uv = uvMap_->get(pId);
        outputMesh_ << "vt " << uv[0] << " " << uv[1] << "\n";

        // Find this UV map's point in _point_links and set its vt value to our
        // current position in the vt list
        pointLinks_.find(pId)->second[1] = vtIndex;

        ++vtIndex;
    }

    // Restore the starting origin
    uvMap_->setOrigin(startingOrigin);
    return EXIT_SUCCESS;
}

// Write the face information: 'f v/vt/vn'
auto OBJWriter::write_faces_() -> int
{
    if (!outputMesh_.is_open() || mesh_->GetNumberOfCells() == 0) {
        return EXIT_FAILURE;
    }
    Logger()->debug("Writing faces...");

    outputMesh_ << "# Faces: " << mesh_->GetNumberOfCells() << "\n";

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
        outputMesh_ << "\n";
    }

    return EXIT_SUCCESS;
}
