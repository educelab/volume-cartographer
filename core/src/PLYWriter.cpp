#include "vc/core/io/PLYWriter.hpp"

#include <cstddef>

#include "vc/core/types/Exceptions.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;
using namespace volcart::io;
namespace fs = volcart::filesystem;

namespace
{
auto PtIntensity(
    std::size_t idx, const UVMap::Pointer& uvMap, const cv::Mat& image)
    -> double
{
    auto uv = uvMap->get(idx);
    int u = cvRound(uv[0] * (image.cols - 1));
    int v = cvRound(uv[1] * (image.rows - 1));
    return image.at<std::uint16_t>(v, u);
}
}  // namespace

///// Output Methods /////
auto PLYWriter::write() -> int
{
    if (not mesh_ or mesh_->GetNumberOfPoints() == 0) {
        throw volcart::IOException("Mesh is empty or null");
    }

    // Open the file stream
    outputMesh_.open(outputPath_.string());
    if (!outputMesh_.is_open()) {
        return EXIT_FAILURE;
    }

    // Capture the starting origin and set origin to what PLY reader needs
    UVMap::Origin startingOrigin{UVMap::Origin::TopLeft};
    if (uvMap_) {
        startingOrigin = uvMap_->origin();
        uvMap_->setOrigin(UVMap::Origin::TopLeft);
    }

    write_header_();
    write_vertices_();
    write_faces_();

    outputMesh_.close();

    // Restore the starting origin
    if (uvMap_) {
        uvMap_->setOrigin(startingOrigin);
    }

    return EXIT_SUCCESS;
}

// Write our custom header
auto PLYWriter::write_header_() -> int
{
    if (!outputMesh_.is_open()) {
        return EXIT_FAILURE;
    }

    outputMesh_ << "ply" << '\n';
    outputMesh_ << "format ascii 1.0" << '\n';
    outputMesh_ << "comment VC PLY Exporter v1.0" << '\n';

    // Vertex Info for Header
    outputMesh_ << "element vertex " << mesh_->GetNumberOfPoints() << '\n';
    outputMesh_ << "property float x" << '\n';
    outputMesh_ << "property float y" << '\n';
    outputMesh_ << "property float z" << '\n';
    outputMesh_ << "property float nx" << '\n';
    outputMesh_ << "property float ny" << '\n';
    outputMesh_ << "property float nz" << '\n';

    // Color info for vertices
    if ((not texture_.empty() and uvMap_ and not uvMap_->empty()) or
        not vcolors_.empty()) {
        outputMesh_ << "property uchar red" << '\n';
        outputMesh_ << "property uchar green" << '\n';
        outputMesh_ << "property uchar blue" << '\n';
    }

    // Face Info for Header
    if (mesh_->GetNumberOfCells() != 0) {
        outputMesh_ << "element face " << mesh_->GetNumberOfCells() << '\n';
        outputMesh_ << "property list uchar int vertex_indices" << '\n';
    }

    // End header
    outputMesh_ << "end_header" << '\n';

    return EXIT_SUCCESS;
}

// Write the vertex information: 'x y z nx ny nz'
auto PLYWriter::write_vertices_() -> int
{
    if (!outputMesh_.is_open() || mesh_->GetNumberOfPoints() == 0) {
        return EXIT_FAILURE;
    }
    Logger()->info("Writing vertices...");

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
        if (not texture_.empty() and uvMap_ and not uvMap_->empty()) {
            // Get the intensity for this point from the texture. If it doesn't
            // exist, set to 0.
            double intensity{0};
            if (uvMap_->contains(point.Index())) {
                intensity = PtIntensity(point.Index(), uvMap_, texture_);
                intensity = cvRound(intensity * 255.0 / 65535.0);
            }

            auto i = static_cast<int>(intensity);
            outputMesh_ << " " << i << " " << i << " " << i;
        } else if (not vcolors_.empty()) {
            float val = vcolors_.at(point.Index());
            auto i = static_cast<int>(val * 255.F / 65535.F);
            outputMesh_ << " " << i << " " << i << " " << i;
        }

        outputMesh_ << '\n';
    }

    return EXIT_SUCCESS;
}

// Write the face information: 'n#-of-verts v1 v1 ... vn'
auto PLYWriter::write_faces_() -> int
{
    if (!outputMesh_.is_open() || mesh_->GetNumberOfCells() == 0) {
        return EXIT_FAILURE;
    }
    Logger()->info("Writing faces...");

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
        outputMesh_ << '\n';
    }

    return EXIT_SUCCESS;
}

void PLYWriter::setVertexColors(const std::vector<std::uint16_t>& c)
{
    vcolors_ = c;
}

PLYWriter::PLYWriter(fs::path outputPath, ITKMesh::Pointer mesh)
    : outputPath_{std::move(outputPath)}, mesh_{std::move(mesh)}
{
}

PLYWriter::PLYWriter(
    fs::path outputPath, ITKMesh::Pointer mesh, cv::Mat texture)
    : outputPath_{std::move(outputPath)}
    , mesh_{std::move(mesh)}
    , texture_{std::move(texture)}
{
}
void PLYWriter::setPath(const filesystem::path& path) { outputPath_ = path; }
void PLYWriter::setMesh(ITKMesh::Pointer mesh) { mesh_ = std::move(mesh); }
void PLYWriter::setUVMap(UVMap::Pointer uvMap) { uvMap_ = std::move(uvMap); }
void PLYWriter::setTexture(cv::Mat texture) { texture_ = std::move(texture); }
