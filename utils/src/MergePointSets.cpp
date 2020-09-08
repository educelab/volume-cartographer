#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/range/iterator_range.hpp>

#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/util/Logging.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

using psio = vc::PointSetIO<cv::Vec3d>;
using Voxel = cv::Vec3d;

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
        ("input,i", po::value<std::string>()->required(),
         "Path to a directory containing all individual pointsets")
        ("output,o", po::value<std::string>()->required(),
         "Path for the output merged pointset");
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

    fs::path inputPath = parsed["input"].as<std::string>();
    fs::path outputPath = parsed["output"].as<std::string>();

    if (!fs::is_directory(inputPath)) {
        vc::logger->error("Must provide a directory as input.");
        return EXIT_FAILURE;
    }

    // Main cloud will hold the merged result
    vc::PointSet<Voxel> cloud;

    // Read all vcps files in the directory:
    for (auto& file :
         boost::make_iterator_range(fs::directory_iterator(inputPath), {})) {
        if (fs::extension(file) != ".vcps") {
            continue;
        }
        // Load the file
        vc::logger->info("Loading file...");
        auto tmpCloud = psio::ReadPointSet(file);
        vc::logger->info("Loaded PointSet with {} points", tmpCloud.size());

        // Add this smaller cloud to the main cloud
        // TODO: it would be nice if we could eliminate the overlap between
        // segmentations and duplicates...
        cloud.append(tmpCloud);
    }

    psio::WritePointSet(outputPath / "merged_pointset.vcps", cloud);
}