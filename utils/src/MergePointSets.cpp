#include <unordered_set>

#include <algorithm>
#include <cctype>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "vc/core/io/FileExtensionFilter.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/util/HashFunctions.hpp"
#include "vc/core/util/Logging.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

// TODO: make this dynamic (int or double should work)
using Voxel = cv::Vec3d;
using VoxelHash = vc::Vec3Hash<Voxel>;
using psio = vc::PointSetIO<Voxel>;

/*
 * Does a very simple merge--merges all .vcps files in a certain directory.
 * Merge pointsets can prune unnecessary parts of a segmentation if
 * the vcps file is named: LAST_SLICE_NUM.vcps. Otherwise
 * pruning will fail. TODO: a regex approach would be better
 * */
int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description all("Usage");
    all.add_options()
        ("help,h", "Show this message")
        ("input-path,i", po::value<std::vector<std::string>>()->required(),
            "Path to a pointset to be merged. May be specified one or more "
            "times. If path is a directory, the directory will be "
            "non-recursively searched for .vcps files.")
        ("output-path,o", po::value<std::string>()->required(),
            "Path for the output merged pointset")
        ("prune,p", "Prune each pointset using its name (name the vcps file the "
            "last slice you want to keep)")
        ("overwrite-overlap", "Overwrite overlapping z-regions with the most recent ps");
    // clang-format on

    // parsed will hold the values of all parsed options as a Map
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") || argc < 2) {
        std::cout << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        vc::logger->error(e.what());
        return EXIT_FAILURE;
    }

    auto inputPaths = parsed["input-path"].as<std::vector<std::string>>();
    fs::path outputPath = parsed["output-path"].as<std::string>();

    // Resolve all provided paths
    std::vector<fs::path> resolvedPaths;
    for (const fs::path p : inputPaths) {
        // Skip non-existent files
        if (not fs::exists(p)) {
            vc::logger->warn("File does not exist: \"{}\"", p.string());
            continue;
        }

        // Handle regular files
        if (fs::is_regular_file(p)) {
            if (vc::IsFileType(p, {"vcps"})) {
                resolvedPaths.emplace_back(p);
            } else {
                vc::logger->info("Skipping file: \"{}\"", p.string());
            }
        }

        // Recursive expand directories
        else if (fs::is_directory(p)) {
            std::vector<fs::path> dirItems;
            fs::directory_iterator dir(p);
            fs::directory_iterator dirEnd;
            for (; dir != dirEnd; ++dir) {
                if (fs::is_regular_file(*dir)) {
                    if (vc::IsFileType(*dir, {"vcps"})) {
                        dirItems.emplace_back(*dir);
                    } else {
                        vc::logger->info(
                            "Skipping file: \"{}\"", dir->path().string());
                    }
                }
            }
            std::sort(dirItems.begin(), dirItems.end());
            std::copy(
                dirItems.begin(), dirItems.end(),
                std::back_inserter(resolvedPaths));
        }
    }

    // Handle duplicates by using unordered_set
    std::unordered_set<Voxel, VoxelHash> pts;

    // Read all vcps files in the directory:
    for (const auto& p : resolvedPaths) {
        // Load the file
        vc::logger->info("Loading file: \"{}\"", p.string());
        auto tmpCloud = psio::ReadPointSet(p);
        vc::logger->info("Loaded pointset with {} points", tmpCloud.size());

        // Prune as needed
        if (parsed.count("prune")) {
            auto filename = p.stem().string();
            if (std::all_of(filename.begin(), filename.end(), ::isdigit)) {
                int maxSliceNum = std::stoi(filename);

                volcart::PointSet<Voxel> prunedCloud;
                for (auto& pt : tmpCloud) {
                    // Check the z-value. Only add it to prunedCloud if z is
                    // less than the vcps name.
                    if (pt[2] <= maxSliceNum) {
                        prunedCloud.push_back(pt);
                    }
                }
                tmpCloud = prunedCloud;
                vc::logger->info(
                    "Pruned pointset to {} points", tmpCloud.size());
            } else {
                vc::logger->warn(
                    "Filename contains characters other than digits. File will "
                    "not be pruned: \"{}\"",
                    p.string());
            }
        }

        // Overwrite overlap
        if (parsed.count("overwrite-overlap")) {
            auto minMax = std::minmax_element(
                tmpCloud.begin(), tmpCloud.end(),
                [](const auto& l, const auto& r) { return l[2] < r[2]; });
            auto minZ = (*minMax.first)[2];
            auto maxZ = (*minMax.second)[2];
            auto origSize = pts.size();
            for (auto i = pts.begin(), last = pts.end(); i != last;) {
                auto z = (*i)[2];
                if (z >= minZ and z <= maxZ) {
                    i = pts.erase(i);
                } else {
                    ++i;
                }
            }
            vc::logger->info(
                "Removing {} points from overlapping region.",
                origSize - pts.size());
        }

        // Add all the points in the smaller cloud to the set
        auto origSize = pts.size();
        pts.reserve(pts.size() + tmpCloud.size());
        pts.insert(tmpCloud.begin(), tmpCloud.end());
        vc::logger->info("Merged {} new points", pts.size() - origSize);
    }

    // Main cloud will hold the merged result
    vc::PointSet<Voxel> cloud(pts.size());
    std::copy(pts.begin(), pts.end(), std::back_inserter(cloud));
    vc::logger->info("Final pointset size: {} points", cloud.size());

    // Save the merged pointset
    vc::logger->info("Writing final pointset...");
    psio::WritePointSet(outputPath, cloud);
    vc::logger->info("Done.");
}
