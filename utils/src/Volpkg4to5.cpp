//
// Created by Seth Parker on 5/11/17.
//

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include "vc/core/types/VolumePkg.hpp"

namespace fs = boost::filesystem;
namespace vc = volcart;

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " [volpkg]" << std::endl;
        return EXIT_FAILURE;
    }

    // Get volpkg path
    fs::path vpkgPath = argv[1];

    // Copy the current metadata
    vc::Metadata volpkgMeta(vpkgPath / "config.json");

    // Check the version
    int v{0};
    if ((v = volpkgMeta.get<int>("version")) != 4) {
        std::cerr << "Unsupported VolPkg version: " << v << std::endl;
        return EXIT_FAILURE;
    }

    // Add metadata to all of the segmentations
    fs::path seg;
    fs::path segsDir = vpkgPath / "paths";
    auto range =
        boost::make_iterator_range(fs::directory_iterator(segsDir), {});
    for (const auto& entry : range) {
        if (fs::is_directory(entry)) {
            // Get the folder as an fs::path
            seg = entry;

            // Generate basic metadata
            vc::Metadata segMeta;
            segMeta.set("uuid", seg.stem().string());
            segMeta.set("name", seg.stem().string());
            segMeta.set("type", "seg");

            // Link the metadata to the vcps file
            if (fs::exists(seg / "pointset.vcps")) {
                segMeta.set("vcps", "pointset.vcps");
            } else {
                segMeta.set("vcps", std::string{});
            }

            // Save the new metadata
            segMeta.save(entry / "meta.json");
        }
    }

    // Add renders folder
    fs::path rendersDir = vpkgPath / "renders";
    if (!fs::exists(rendersDir)) {
        fs::create_directory(rendersDir);
    } else {
        std::cout << "Package has renders directory: " << vpkgPath << std::endl;
        return EXIT_FAILURE;
    }

    // Try to load as a volpkg
    // Should fail if the constructor can't find new metadata
    try {
        vc::VolumePkg vpkg(argv[1]);
    } catch (const std::exception& e) {
        std::cout << "Failed to load updated VolumePkg: " << e.what()
                  << std::endl;
        return EXIT_FAILURE;
    }

    // Update the version
    volpkgMeta.set("version", 5);
    volpkgMeta.save();

    std::cout << "VolumePkg updated successfully!" << std::endl;
    return EXIT_SUCCESS;
}