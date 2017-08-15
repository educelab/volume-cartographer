#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "vc/core/types/VolumePkg.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 5;

int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path");

    po::options_description all("Usage");
    all.add(required);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") || argc < 3) {
        std::cerr << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Get the parsed options
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();

    ///// Load the volume package /////
    vc::VolumePkg vpkg(volpkgPath);
    if (vpkg.getVersion() != VOLPKG_SUPPORTED_VERSION) {
        std::cerr << "ERROR: Volume package is version " << vpkg.getVersion()
                  << " but this program requires version "
                  << VOLPKG_SUPPORTED_VERSION << "." << std::endl;
        return EXIT_FAILURE;
    }

    ///// VolumePkg /////
    std::cout << std::endl;
    std::cout << " --- VolumePkg ---" << std::endl;
    std::cout << "Name: " << vpkg.getPkgName() << std::endl;
    std::cout << "Material Thickness: " << vpkg.getMaterialThickness() << "um"
              << std::endl;
    std::cout << std::endl;

    ///// List the volumes /////
    std::cout << " --- Volumes ---" << std::endl;
    auto volIds = vpkg.volumeIDs();
    for (auto& id : volIds) {
        auto vol = vpkg.volume(id);
        std::cout << "[" << id << "] " << vol->name() << ", ";
        std::cout << vol->sliceWidth() << "x" << vol->sliceHeight() << "x"
                  << vol->numSlices() << ", ";
        std::cout << vol->voxelSize() << "um/voxel" << std::endl;
    }
    std::cout << std::endl;

    ///// List the segmentations /////
    std::cout << " --- Segmentations ---" << std::endl;
    for (const auto& s : vpkg.segmentationIDs()) {
        auto seg = vpkg.segmentation(s);
        std::cout << "[" << seg->id() << "] " << seg->name() << std::endl;
    }
    std::cout << std::endl;
}