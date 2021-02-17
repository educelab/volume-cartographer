#include "vc/apps/render/RenderIO.hpp"

#include <cstdlib>
#include <iostream>

#include <opencv2/imgcodecs.hpp>

#include "vc/core/io/FileExtensionFilter.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYReader.hpp"
#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/io/TIFFIO.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/ImageConversion.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vcm = volcart::meshing;

extern po::variables_map parsed_;
extern vc::VolumePkg::Pointer vpkg_;
extern vc::Segmentation::Pointer seg_;
extern vc::Volume::Pointer volume_;
extern vc::UVMap parsedUVMap_;

po::options_description GetIOOpts()
{
    // clang-format off
    po::options_description opts("Input/Output Options");
    opts.add_options()
    ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
    ("seg,s", po::value<std::string>(), "Segmentation ID")
    ("input-mesh", po::value<std::string>(), "Path to input OBJ or PLY")
    ("volume", po::value<std::string>(),
        "Volume to use for texturing. Default: Segmentation's associated "
        "volume or the first volume in the volume package.")
    ("output-file,o", po::value<std::string>(),
        "Output file path. If not specified, an OBJ file and texture image "
        "will be placed in the current working directory.")
    ("output-ppm", po::value<std::string>(),
        "Output file path for the generated PPM.")
    ("tiff-floating-point", "When outputting to the TIFF format, save a "
        "floating-point image.");
    // clang-format on

    return opts;
}

vc::ITKMesh::Pointer LoadSegmentation(const vc::Segmentation::Identifier& id)
{
    vc::Logger()->info("Loading segmentation: {}", id);
    try {
        seg_ = vpkg_->segmentation(id);
    } catch (const std::exception& e) {
        vc::Logger()->critical(
            "Cannot load segmentation. Check that the provided ID exists: {}",
            id);
        vc::Logger()->debug(e.what());
        std::exit(EXIT_FAILURE);
    }

    // Load the point cloud
    auto points = seg_->getPointSet();

    // Mesh the point cloud
    vc::Logger()->info("Meshing point set");
    vcm::OrderedPointSetMesher mesher;
    mesher.setPointSet(points);
    return mesher.compute();
}

vc::ITKMesh::Pointer LoadMeshFile(const fs::path& p)
{
    vc::Logger()->info("Loading mesh: {}", p.string());
    // OBJs
    if (vc::io::FileExtensionFilter(p, {"obj"})) {
        vc::io::OBJReader r;
        r.setPath(p);
        r.read();
        try {
            auto texture = r.getTextureMat();
            parsedUVMap_ = r.getUVMap();
            parsedUVMap_.ratio(texture.cols, texture.rows);
        } catch (...) {
            // Do nothing if there's no texture image
        }

        return r.getMesh();
    }

    // PLYs
    else if (vc::io::FileExtensionFilter(p, {"ply"})) {
        vc::io::PLYReader r(p);
        return r.read();
    }

    // Can't load file
    else {
        vc::Logger()->error("Mesh file not of supported type: {}", p.string());
        std::exit(EXIT_FAILURE);
    }
}

void SaveOutput(
    const fs::path& outputPath,
    const vc::ITKMesh::Pointer& mesh,
    vc::Texture texture)
{
    // Convert image depth
    auto tgtDepth{CV_16U};
    auto requestTIFF = vc::IsFileType(outputPath, {"tiff", "tif"});
    auto requestFloat = parsed_.count("tiff-floating-point") > 0;
    if (requestTIFF && requestFloat) {
        tgtDepth = CV_32F;
    } else if (vc::IsFileType(outputPath, {"jpg", "jpeg"})) {
        tgtDepth = CV_8U;
    }
    auto m = vc::QuantizeImage(texture.image(0), tgtDepth);
    texture.setImage(0, m);

    // Write the output
    if (requestTIFF && requestFloat) {
        vc::Logger()->info("Writing to floating-point TIF");
        vc::tiffio::WriteTIFF(outputPath, texture.image(0));
    } else if (vc::IsFileType(outputPath, {"ply"})) {
        vc::Logger()->info("Writing to PLY");
        vc::io::PLYWriter writer(outputPath.string(), mesh, texture);
        writer.write();
    } else if (vc::IsFileType(
                   outputPath, {"png", "jpg", "jpeg", "tiff", "tif"})) {
        vc::Logger()->info("Writing to image");
        cv::imwrite(outputPath.string(), texture.image(0));
    } else if (vc::IsFileType(outputPath, {"obj"})) {
        vc::Logger()->info("Writing to OBJ");
        vc::io::OBJWriter writer;
        writer.setMesh(mesh);
        writer.setUVMap(texture.uvMap());
        writer.setTexture(texture.image(0));
        writer.setPath(outputPath.string());
        writer.write();
    } else {
        vc::Logger()->error(
            "Unrecognized output format: {}", outputPath.extension().string());
    }
}