#include "vc/core/filesystem.hpp"
#include "vc/core/types/Metadata.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/DateTime.hpp"
#include "vc/core/util/Logging.hpp"

namespace fs = volcart::filesystem;
namespace vc = volcart;

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " [volpkg]\n";
        return EXIT_FAILURE;
    }

    // VolPkg Path
    const fs::path path = argv[1];
    std::cout << "VolPkg Upgrade: " << path << "\n";

    // Copy the current metadata
    vc::Metadata origMeta(path / "config.json");

    // Version check
    if (origMeta.get<int>("version") == vc::VOLPKG_VERSION_LATEST) {
        try {
            const vc::VolumePkg vpkg(path);
            std::cout << "VolumePkg is up-to-date.\n";
            return EXIT_SUCCESS;
        } catch (const std::exception& e) {
            std::cerr << "VolumePkg is latest version, but failed to load: "
                      << e.what() << "\n";
            return EXIT_FAILURE;
        }
    }

    // Upgrade tasks
    vc::logging::SetLogLevel("debug");
    vc::VolumePkg::Upgrade(path, vc::VOLPKG_VERSION_LATEST, true);

    // Try to load as a volpkg
    try {
        const vc::VolumePkg vpkg(path);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load updated VolumePkg: " << e.what() << "\n";
        std::cerr << "Restoring original metadata.\n";
        origMeta.save();
    }

    // Done
    std::cout << "Upgrades complete.\n";
}
