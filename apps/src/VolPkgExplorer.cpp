#include <iostream>

#include <boost/program_options.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/graph.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

// Volpkg version required by this app
static constexpr int VOLPKG_MIN_VERSION = 6;

auto main(int argc, char* argv[]) -> int
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
    if (parsed.count("help") > 0 || argc < 3) {
        std::cout << all << "\n";
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        vc::Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    // Register Volume Cartographer Nodes
    vc::RegisterNodes();

    // Get the parsed options
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();

    ///// Load the volume package /////
    vc::VolumePkg vpkg(volpkgPath);
    if (vpkg.version() < VOLPKG_MIN_VERSION) {
        vc::Logger()->error(
            "Volume package is version {} but this program requires version "
            "{}.",
            vpkg.version(), VOLPKG_MIN_VERSION);
        return EXIT_FAILURE;
    }

    ///// VolumePkg /////
    std::cout << "\n";
    std::cout << " --- VolumePkg ---\n";
    std::cout << "Name: " << vpkg.name() << "\n";
    std::cout << "Material Thickness: " << vpkg.materialThickness() << "um\n\n";

    ///// List the volumes /////
    std::cout << " --- Volumes ---\n";
    for (const auto& id : vpkg.volumeIDs()) {
        auto vol = vpkg.volume(id);
        std::cout << "[" << id << "] " << vol->name() << ", ";
        std::cout << vol->sliceWidth() << "x" << vol->sliceHeight() << "x"
                  << vol->numSlices() << ", ";
        std::cout << vol->voxelSize() << "um/voxel\n";
    }
    std::cout << "\n";

    ///// List the segmentations /////
    std::cout << " --- Segmentations ---\n";
    for (const auto& s : vpkg.segmentationIDs()) {
        auto seg = vpkg.segmentation(s);
        std::cout << "[" << seg->id() << "] " << seg->name();
        if (seg->hasVolumeID()) {
            std::cout << ", associated volume: " << seg->getVolumeID();
        }
        std::cout << "\n";
    }
    std::cout << "\n";

    ///// List the renders /////
    std::cout << " --- Renders ---\n";
    for (const auto& r : vpkg.renderIDs()) {
        auto render = vpkg.render(r);
        std::cout << "[" << render->id() << "] " << render->name();
        std::cout << ", number of nodes: " << render->graph()->size();
        std::cout << "\n";
    }
    std::cout << "\n";

    ///// List the transforms /////
    std::cout << " --- Transforms ---\n";
    for (const auto& t : vpkg.transformIDs()) {
        auto tfm = vpkg.transform(t);
        std::cout << "[" << t << "] type: " << tfm->type();
        std::cout << ", source: " << tfm->source();
        std::cout << ", target: " << tfm->target();
        std::cout << "\n";
    }
    std::cout << "\n";
}
