#include <unordered_set>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/range/iterator_range.hpp>

#include "vc/core/io/FileExtensionFilter.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/util/Logging.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

// TODO: make this dynamic (int or double should work)
using psio = vc::PointSetIO<cv::Vec3d>;
using Voxel = cv::Vec3d;

// Hash taken from ThinnedFloodFillSegmentation
struct VoxelHash {
    size_t operator()(const Voxel& v) const
    {
        // Hash from:
        // https://dmauro.com/post/77011214305/a-hashing-function-for-x-y-z-coordinates
        auto max = std::max({v[0], v[1], v[2]});
        size_t hash = (max * max * max) + (2 * max * v[2]) + v[2];
        if (max == v[2]) {
            auto val = std::max({v[0], v[1]});
            hash += val * val;
        }
        if (v[1] >= v[0]) {
            hash += v[0] + v[1];
        } else {
            hash += v[1];
        }
        return hash;
    }
};

/*
 * Does a very simple merge--merges all .vcps files in a certain directory.
 * Does not remove 'overlap' between segmentations, though this would be
 * helpful.
 * */
int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description all("Usage");
    all.add_options()
        ("help,h", "Show this message")
        ("input-dir,i", po::value<std::string>()->required(),
            "Path to a directory containing all individual pointsets")
        ("output-dir,o", po::value<std::string>()->required(),
            "Path to a directory to store the output merged pointset")
        ("prune,p", "Prune each pointset using its name (name the vcps file the "
            "last slice you want to keep)");
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

    fs::path inputPath = parsed["input-dir"].as<std::string>();
    fs::path outputPath = parsed["output-dir"].as<std::string>();

    if (!fs::is_directory(inputPath)) {
        vc::logger->error("Must provide a directory as input.");
        return EXIT_FAILURE;
    }

    // Handle duplicates by using unordered_set
    std::unordered_set<Voxel, VoxelHash> pts;

    // Read all vcps files in the directory:
    for (const auto& file :
         boost::make_iterator_range(fs::directory_iterator(inputPath))) {

        // Get file as fs::path
        const auto& fpath = file.path();

        // Silent skip directories
        if (fs::is_directory(fpath)) {
            continue;
        }

        // Skip non-pointset files
        if (not vc::IsFileType(fpath, {"vcps"})) {
            vc::logger->info("Skipping file: \"{}\"", fpath.string());
            continue;
        }

        // Load the file
        vc::logger->info("Loading file: \"{}\"", fpath.string());
        auto tmpCloud = psio::ReadPointSet(fpath);
        vc::logger->info("Loaded pointset with {} points", tmpCloud.size());

        // Prune as needed
        if (parsed.count("prune")) {
            // Match the regex
            static const std::regex SLICE_REG{".*?(\\d+)+\\.vcps$"};
            std::smatch matches;
            if (not std::regex_match(
                    fs::path(file).string(), matches, SLICE_REG)) {
                vc::logger->warn(
                    "Filename does not match pruning pattern. File will not be "
                    "pruned: \"{}\"",
                    fpath.string());
                continue;
            }
            else {
                // Get the digits
                // First match is the whole regex
                // Second match is the capture group of one or more digits: \\d+
                int maxSliceNum = std::stoi(matches[1].str());

                volcart::PointSet<Voxel> prunedCloud;
                for (auto& pt : tmpCloud) {
                    // Check the z-value. Only add it to prunedCloud if z is less than the vcps name.
                    if (pt[2] <= maxSliceNum) {
                        prunedCloud.push_back(pt);
                    }
                }
                tmpCloud = prunedCloud;
                vc::logger->info("Pruned pointset to {} points", tmpCloud.size());
            }
        }
        // Add all the points in the smaller cloud to the set
        auto origSize = pts.size();
        pts.insert(tmpCloud.begin(), tmpCloud.end());
        vc::logger->info("Merged {} new points", pts.size() - origSize);
    }

    // Main cloud will hold the merged result
    vc::PointSet<Voxel> cloud(pts.size());
    std::copy(pts.begin(), pts.end(), std::back_inserter(cloud));
    vc::logger->info("Final pointset size: {} points", cloud.size());

    // Save the merged pointset
    psio::WritePointSet(outputPath / "merged_pointset.vcps", cloud);
}