//
// Created by Seth Parker on 5/11/17.
//

#include <boost/filesystem.hpp>

#include "vc/core/types/Metadata.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/util/DateTime.hpp"

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
    vc::Metadata oldMeta(vpkgPath / "config.json");

    // Check the version
    int v{0};
    if ((v = oldMeta.get<int>("version")) != 3) {
        std::cerr << "Unsupported VolPkg version: " << v << std::endl;
        return EXIT_FAILURE;
    }

    // Write the new volpkg metadata
    vc::Metadata vpkgMeta;
    vpkgMeta.set("version", 4);
    vpkgMeta.set("name", oldMeta.get<std::string>("volumepkg name"));
    vpkgMeta.set("materialthickness", oldMeta.get<double>("materialthickness"));
    vpkgMeta.save(vpkgPath / "config.json");

    // Make the "volumes" directory
    fs::path volumesDir = vpkgPath / "volumes";
    if (!fs::exists(volumesDir)) {
        fs::create_directory(volumesDir);
    }

    // Setup a new Volume name and make a new folder for it
    // Move the slices
    auto id = vc::DateTime();
    auto newVolDir = volumesDir / id;
    fs::rename(vpkgPath / "slices", newVolDir);

    // Setup and save the metadata to the new Volume folder
    vc::Metadata volMeta;
    volMeta.set("uuid", id);
    volMeta.set("name", id);
    volMeta.set("width", oldMeta.get<int>("width"));
    volMeta.set("height", oldMeta.get<int>("height"));
    volMeta.set("slices", oldMeta.get<int>("number of slices"));
    volMeta.set("voxelsize", oldMeta.get<double>("voxelsize"));
    volMeta.set("min", oldMeta.get<double>("min"));
    volMeta.set("max", oldMeta.get<double>("max"));
    volMeta.save(newVolDir / "meta.json");
}