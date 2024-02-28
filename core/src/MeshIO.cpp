#include "vc/core/io/MeshIO.hpp"

#include "vc/core/io/FileFilters.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYReader.hpp"
#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;
using namespace volcart::io;

auto volcart::ReadMesh(const filesystem::path& path) -> MeshReaderResult
{
    MeshReaderResult result;
    // OBJs
    if (IsFileType(path, {"obj"})) {
        OBJReader r;
        r.setPath(path);
        result.mesh = r.read();
        if (not r.getTextureMat().empty()) {
            result.texture = r.getTextureMat();
        }
        if (not r.getUVMap()->empty()) {
            result.uv = r.getUVMap();
        }
    }

    // PLYs
    else if (IsFileType(path, {"ply"})) {
        PLYReader r;
        r.setPath(path);
        result.mesh = r.read();
    }

    // Can't load file
    else {
        auto msg = "Mesh file not of supported type: " + path.string();
        throw std::invalid_argument(msg);
    }
    return result;
}

void volcart::WriteMesh(
    const filesystem::path& path,
    const ITKMesh::Pointer& mesh,
    const UVMap::Pointer& uv,
    const cv::Mat& texture,
    const MeshWriterOpts& opts)
{
    if (IsFileType(path, {"obj"})) {
        OBJWriter writer;
        writer.setPath(path);
        writer.setMesh(mesh);
        writer.setTextureFormat(opts.imgFmt);
        if (uv) {
            writer.setUVMap(uv);
            writer.setTexture(texture);
        }
        writer.write();
    }

    else if (IsFileType(path, {"ply"})) {
        PLYWriter writer;
        writer.setPath(path);
        writer.setMesh(mesh);
        // TODO: Add texture writing support back
        writer.write();
    }
}
