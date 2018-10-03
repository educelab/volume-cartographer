// render.cpp

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/MemorySizeStringParser.hpp"
#include "vc/external/GetMemorySize.hpp"
#include "vc/meshing/ScaleMesh.hpp"
#include "vc/texturing/ScaleMarkerGenerator.hpp"

// App includes
#include "apps/RenderGeneral.hpp"
#include "apps/RenderIO.hpp"
#include "apps/RenderMeshing.hpp"
#include "apps/RenderTexturing.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vcm = volcart::meshing;
namespace vct = volcart::texturing;

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 6;

// Globals
po::variables_map parsed_;
vc::VolumePkg::Pointer vpkg_;
vc::Segmentation::Pointer seg_;
vc::Volume::Pointer volume_;
vc::UVMap parsedUVMap_;

int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    po::options_description all("Usage");
    all.add(GetGeneralOpts())
        .add(GetIOOpts())
        .add(GetMeshingOpts())
        .add(GetUVOpts())
        .add(GetFilteringOpts())
        .add(GetCompositeOpts())
        .add(GetIntegralOpts())
        .add(GetPostProcessOpts());

    // Parse the cmd line
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed_);

    // Show the help message
    if (parsed_.count("help") || argc < 5) {
        std::cerr << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed_);
    } catch (po::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    ///// Load the volume package /////
    fs::path volpkgPath = parsed_["volpkg"].as<std::string>();
    vpkg_ = vc::VolumePkg::New(volpkgPath);
    if (vpkg_->version() != VOLPKG_SUPPORTED_VERSION) {
        std::cerr << "ERROR: Volume Package is version " << vpkg_->version()
                  << " but this program requires version "
                  << VOLPKG_SUPPORTED_VERSION << "." << std::endl;
        return EXIT_FAILURE;
    }

    ///// Load the Segmentation or the Mesh file /////
    std::string filenameBase;
    vc::ITKMesh::Pointer input;
    bool loadSeg = parsed_.count("seg") > 0;
    bool loadMesh = parsed_.count("input-mesh") > 0;
    if (!loadSeg && !loadMesh) {
        std::cerr << "ERROR: Did not provide required flag Segmentation ID "
                     "(--seg) or mesh path (--input-mesh)"
                  << std::endl;
        return EXIT_FAILURE;
    } else if (loadSeg && loadMesh) {
        std::cerr << "ERROR: Provided Segmentation ID and mesh path as input. "
                     "Only one input is supported."
                  << std::endl;
        return EXIT_FAILURE;
    }

    if (loadSeg) {
        auto segID = parsed_["seg"].as<std::string>();
        input = LoadSegmentation(segID);
        filenameBase = segID;
    } else {
        fs::path meshPath = parsed_["input-mesh"].as<std::string>();
        input = LoadMeshFile(meshPath);
        filenameBase = meshPath.stem().string();
    }

    // Setup the output file
    fs::path outputPath;
    if (parsed_.count("output-file")) {
        outputPath = parsed_["output-file"].as<std::string>();
    } else {
        outputPath = filenameBase + "_render.obj";
    }

    ///// Load the Volume /////
    try {
        if (parsed_.count("volume")) {
            volume_ = vpkg_->volume(parsed_["volume"].as<std::string>());
        } else if (loadSeg && seg_->hasVolumeID()) {
            volume_ = vpkg_->volume(seg_->getVolumeID());
        } else {
            volume_ = vpkg_->volume();
        }
    } catch (const std::exception& e) {
        std::cerr << "Cannot load volume. ";
        std::cerr << "Please check that the Volume Package has volumes and "
                     "that the volume ID is correct."
                  << std::endl;
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Set the cache size
    size_t cacheBytes;
    if (parsed_.count("cache-memory-limit")) {
        auto cacheSizeOpt = parsed_["cache-memory-limit"].as<std::string>();
        cacheBytes = vc::MemorySizeStringParser(cacheSizeOpt);
    } else {
        cacheBytes = SystemMemorySize() / 2;
    }
    volume_->setCacheMemoryInBytes(cacheBytes);
    std::cout << "Volume Cache :: ";
    std::cout << "Capacity: " << volume_->getCacheCapacity() << " || ";
    std::cout << "Size: " << vc::BytesToMemorySizeString(cacheBytes);
    std::cout << std::endl;

    //// Scale the mesh /////
    if (parsed_.count("scale-mesh") > 0) {
        std::cout << "Scaling mesh..." << std::endl;
        auto scaleFactor = parsed_["scale-mesh"].as<double>();
        auto scaled = vc::ITKMesh::New();
        vcm::ScaleMesh(input, scaled, scaleFactor);
        input = scaled;
    }

    ///// Resample and smooth the mesh /////
    auto needResample = loadSeg || parsed_.count("enable-mesh-resampling");
    if (needResample) {
        input = ResampleMesh(input);
    }

    ///// Flattening /////
    auto uvMap = FlattenMesh(input, needResample);

    ///// Texturing /////
    auto texture = TextureMesh(input, uvMap);

    ///// Post-processing /////
    RenderPostProcess(texture);

    ///// Saving /////
    SaveOutput(outputPath, input, texture);

    return EXIT_SUCCESS;
}  // end main
