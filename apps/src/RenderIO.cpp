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
#include "vc/meshing/OrderedPointSetMesher.hpp"

namespace fs = boost::filesystem;
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
    try {
        seg_ = vpkg_->segmentation(id);
    } catch (const std::exception& e) {
        std::cerr << "Cannot load segmentation. ";
        std::cerr << "Please check the provided ID: " << id << std::endl;
        std::cerr << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    // Load the point cloud
    std::cout << "Loading segmentation..." << std::endl;
    auto points = seg_->getPointSet();

    // Mesh the point cloud
    std::cout << "Meshing point set..." << std::endl;
    vcm::OrderedPointSetMesher mesher;
    mesher.setPointSet(points);
    return mesher.compute();
}

vc::ITKMesh::Pointer LoadMeshFile(const fs::path& p)
{
    std::cout << "Loading mesh..." << std::endl;
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
        std::cerr << "ERROR: Mesh file not of supported type: ";
        std::cerr << p << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

void SaveOutput(
    const fs::path& outputPath,
    const vc::ITKMesh::Pointer& mesh,
    vc::Texture texture)
{
    // Convert to/from floating point
    auto requestTIFF = vc::io::FileExtensionFilter(outputPath, {"tiff", "tif"});
    auto requestFloat = parsed_.count("tiff-floating-point") > 0;
    if (requestTIFF && requestFloat) {
        auto m = vc::QuantizeImage(texture.image(0), CV_32F);
        texture.setImage(0, m);
    } else {
        auto m = texture.image(0);
        if (m.depth() == CV_32F || m.depth() == CV_64F) {
            m = vc::QuantizeImage(m, CV_16U);
            texture.setImage(0, m);
        }
    }

    // Write the output
    if (requestTIFF && requestFloat) {
        std::cout << "Writing to floating-point TIF ..." << std::endl;
        vc::tiffio::WriteTIFF(outputPath, texture.image(0));
    } else if (vc::io::FileExtensionFilter(outputPath, {"ply"})) {
        std::cout << "Writing to PLY..." << std::endl;
        vc::io::PLYWriter writer(outputPath.string(), mesh, texture);
        writer.write();
    } else if (vc::io::FileExtensionFilter(
                   outputPath, {"png", "jpg", "jpeg", "tiff", "tif"})) {
        std::cout << "Writing to image..." << std::endl;
        cv::imwrite(outputPath.string(), texture.image(0));
    } else if (vc::io::FileExtensionFilter(outputPath, {"obj"})) {
        std::cout << "Writing to OBJ..." << std::endl;
        vc::io::OBJWriter writer;
        writer.setMesh(mesh);
        writer.setUVMap(texture.uvMap());
        writer.setTexture(texture.image(0));
        writer.setPath(outputPath.string());
        writer.write();
    } else {
        std::cerr << "Unrecognized output format: " << outputPath.extension();
        std::cerr << std::endl;
    }
}