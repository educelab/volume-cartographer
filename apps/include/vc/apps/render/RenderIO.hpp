#pragma once

#include <boost/program_options.hpp>
#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/Segmentation.hpp"
#include "vc/core/types/UVMap.hpp"

/** Get IO options */
boost::program_options::options_description GetIOOpts();

/** Load a Segmentation from a VolumePkg and convert it to a mesh */
volcart::ITKMesh::Pointer LoadSegmentation(
    const volcart::Segmentation::Identifier& id);

/** Load a mesh file from a path */
volcart::ITKMesh::Pointer LoadMeshFile(const volcart::filesystem::path& p);

/** Save the generated texture and mesh to a path */
void SaveOutput(
    const volcart::filesystem::path& outputPath,
    const volcart::ITKMesh::Pointer& mesh,
    const volcart::UVMap::Pointer& uvMap,
    const std::vector<cv::Mat>& texture);