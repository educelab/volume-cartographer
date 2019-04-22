#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include "vc/core/types/Metadata.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/DateTime.hpp"

namespace fs = boost::filesystem;
namespace vc = volcart;

void volpkgV3ToV4(const fs::path& path);
void volpkgV4ToV5(const fs::path& path);
void volpkgV5ToV6(const fs::path& path);

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " [volpkg]" << std::endl;
        return EXIT_FAILURE;
    }

    // VolPkg Path
    fs::path path = argv[1];
    std::cout << "VolPkg Upgrade: " << path << std::endl;

    // Copy the current metadata
    vc::Metadata origMeta(path / "config.json");

    // Version check
    if (origMeta.get<int>("version") == vc::VOLPKG_VERSION_LATEST) {
        try {
            vc::VolumePkg vpkg(path);
            std::cout << "VolumePkg is up-to-date." << std::endl;
            return EXIT_SUCCESS;
        } catch (const std::exception& e) {
            std::cerr << "VolumePkg is latest version, but failed to load: "
                      << e.what() << "\n";
            return EXIT_FAILURE;
        }
    }

    // Upgrade tasks
    volpkgV3ToV4(path);
    volpkgV4ToV5(path);
    volpkgV5ToV6(path);

    // Try to load as a volpkg
    try {
        vc::VolumePkg vpkg(path);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load updated VolumePkg: " << e.what() << "\n";
        std::cerr << "Restoring original metadata." << std::endl;
        origMeta.save();
    }

    // Done
    std::cout << "Upgrades complete." << std::endl;
}

void volpkgV3ToV4(const fs::path& path)
{
    // Copy the current metadata
    vc::Metadata oldMeta(path / "config.json");

    // Nothing to do
    if (oldMeta.get<int>("version") != 3) {
        return;
    }
    std::cout << "Upgrading to version 4..." << std::endl;

    // Write the new volpkg metadata
    vc::Metadata vpkgMeta;
    vpkgMeta.set("version", 4);
    vpkgMeta.set("name", oldMeta.get<std::string>("volumepkg name"));
    vpkgMeta.set("materialthickness", oldMeta.get<double>("materialthickness"));
    vpkgMeta.save(path / "config.json");

    // Make the "volumes" directory
    fs::path volumesDir = path / "volumes";
    if (!fs::exists(volumesDir)) {
        fs::create_directory(volumesDir);
    }

    // Setup a new Volume name and make a new folder for it
    // Move the slices
    auto id = vc::DateTime();
    auto newVolDir = volumesDir / id;
    fs::rename(path / "slices", newVolDir);

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

void volpkgV4ToV5(const fs::path& path)
{
    // Copy the current metadata
    vc::Metadata volpkgMeta(path / "config.json");

    // Nothing to do check
    if (volpkgMeta.get<int>("version") != 4) {
        return;
    }
    std::cout << "Upgrading to version 5..." << std::endl;

    // Add metadata to all of the segmentations
    fs::path seg;
    fs::path segsDir = path / "paths";
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
    fs::path rendersDir = path / "renders";
    if (!fs::exists(rendersDir)) {
        fs::create_directory(rendersDir);
    } else {
        std::cout << "Package has renders directory: " << path << std::endl;
    }

    // Update the version
    volpkgMeta.set("version", 5);
    volpkgMeta.save();
}

void volpkgV5ToV6(const fs::path& path)
{
    // Copy the current metadata
    vc::Metadata volpkgMeta(path / "config.json");

    // Nothing to do check
    if (volpkgMeta.get<int>("version") != 5) {
        return;
    }
    std::cout << "Upgrading to version 6..." << std::endl;

    // Add metadata to all of the volumes
    fs::path vol;
    fs::path volsDir = path / "volumes";
    auto range =
        boost::make_iterator_range(fs::directory_iterator(volsDir), {});
    for (const auto& entry : range) {
        if (fs::is_directory(entry)) {
            // Get the folder as an fs::path
            vol = entry;

            // Generate basic metadata
            vc::Metadata volMeta(entry / "meta.json");
            if (!volMeta.hasKey("uuid")) {
                volMeta.set("uuid", vol.stem().string());
            }
            if (!volMeta.hasKey("name")) {
                volMeta.set("name", vol.stem().string());
            }
            if (!volMeta.hasKey("type")) {
                volMeta.set("type", "vol");
            }

            // Save the new metadata
            volMeta.save();
        }
    }

    // Update the version
    volpkgMeta.set("version", 6);
    volpkgMeta.save();
}
