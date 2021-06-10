// render.cpp
#include <iostream>
#include <string>
#include <vector>

#include <boost/program_options.hpp>

#include "vc/app_support/GetMemorySize.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/MemorySizeStringParser.hpp"
#include "vc/meshing/ScaleMesh.hpp"

// App includes
#include "vc/app_support/GeneralOptions.hpp"
#include "vc/apps/render/RenderIO.hpp"
#include "vc/apps/render/RenderMeshing.hpp"
#include "vc/apps/render/RenderTexturing.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vcm = volcart::meshing;

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
        .add(GetThicknessOpts())
        .add(GetPostProcessOpts());

    // Parse the cmd line
    try {
        po::store(
            po::command_line_parser(argc, argv).options(all).run(), parsed_);
    } catch (const po::error& e) {
        vc::Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    // Show the help message
    if (parsed_.count("help") || argc < 5) {
        std::cout << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed_);
    } catch (po::error& e) {
        vc::Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    // Set logging level
    auto logLevel = parsed_["log-level"].as<std::string>();
    std::transform(
        logLevel.begin(), logLevel.end(), logLevel.begin(), ::tolower);
    vc::logging::SetLogLevel(logLevel);

    ///// Load the volume package /////
    fs::path volpkgPath = parsed_["volpkg"].as<std::string>();
    vc::Logger()->info("Loading VolumePkg: {}", volpkgPath.string());
    try {
        vpkg_ = vc::VolumePkg::New(volpkgPath);
    } catch (const std::exception& e) {
        vc::Logger()->critical(e.what());
        return EXIT_FAILURE;
    }

    if (vpkg_->version() != VOLPKG_SUPPORTED_VERSION) {
        vc::Logger()->error(
            "Volume Package is version {} but this program requires version {}",
            vpkg_->version(), VOLPKG_SUPPORTED_VERSION);
        return EXIT_FAILURE;
    }

    ///// Load the Segmentation or the Mesh file /////
    std::string filenameBase;
    vc::ITKMesh::Pointer input;
    bool loadSeg = parsed_.count("seg") > 0;
    bool loadMesh = parsed_.count("input-mesh") > 0;
    if (!loadSeg && !loadMesh) {
        vc::Logger()->error(
            "Did not provide required flag Segmentation ID (--seg) or mesh "
            "path (--input-mesh)");
        return EXIT_FAILURE;
    } else if (loadSeg && loadMesh) {
        vc::Logger()->error(
            "ERROR: Provided Segmentation ID and mesh path as input. Only one "
            "input is supported.");
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
    vc::Logger()->info(
        "Input surface mesh :: Vertices {} || Faces: {}",
        input->GetNumberOfPoints(), input->GetNumberOfCells());

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
        vc::Logger()->error(
            "Cannot load volume. Please check that the Volume Package has "
            "volumes and that the volume ID is correct: {}",
            e.what());
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
    vc::Logger()->info(
        "Volume Cache :: Capacity: {} || Size: {}", volume_->getCacheCapacity(),
        vc::BytesToMemorySizeString(cacheBytes));

    //// Scale the mesh /////
    if (parsed_.count("scale-mesh") > 0) {
        vc::Logger()->info("Scaling mesh");
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
