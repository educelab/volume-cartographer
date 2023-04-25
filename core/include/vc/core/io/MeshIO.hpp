#pragma once

#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart
{

/** @brief Result for ReadMesh */
struct MeshReaderResult {
    /** Mesh */
    ITKMesh::Pointer mesh;
    /** UV map, if loaded a textured mesh. nullptr if not textured */
    UVMap::Pointer uv;
    /** Texture image, if loaded a textured mesh. nullptr if not textured */
    cv::Mat texture;
};

/**
 * @brief Read a mesh from a file
 *
 * Uses the file extension to determine which mesh reader is used (e.g.
 * OBJReader, PLYReader, etc.).
 *
 * @param path Input file path
 */
auto ReadMesh(const filesystem::path& path) -> MeshReaderResult;

/** @brief General options for WriteMesh */
struct MeshWriterOpts {
    std::string imgFmt{"tif"};
};

/**
 * @brief Write a mesh to a file
 *
 * Uses the provided file extension to determine which mesh writer is used
 * (e.g. OBJWriter, PLYWriter, etc.).
 *
 * @param path Output file path
 * @param mesh Mesh to write
 * @param uv UVMap for the provided mesh. Required to write a textured mesh.
 * @param texture Texture image mapped by the provided UV Map. Required to
 * write a textured mesh.
 */
void WriteMesh(
    const filesystem::path& path,
    const ITKMesh::Pointer& mesh,
    const UVMap::Pointer& uv = nullptr,
    const cv::Mat& texture = cv::Mat(),
    const MeshWriterOpts& opts = {});
}  // namespace volcart