#include <boost/program_options.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
using vc::enumerate;
using vc::range;

using psio = vc::PointSetIO<cv::Vec3d>;
using PointSet = vc::OrderedPointSet<cv::Vec3d>;

auto main(int argc, char* argv[]) -> int
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description all("Usage");
    all.add_options()
        ("help,h", "Show this message")
        ("input,i", po::value<std::string>()->required(),
             "Path to the input PointSet")
        ("output,o", po::value<std::string>(), "Path to the output PointSet")
        ("verbose,v", "Print verbose information");
    // clang-format on

    // parsed will hold the values of all parsed options as a Map
    po::variables_map args;
    po::store(po::command_line_parser(argc, argv).options(all).run(), args);

    // Show the help message
    if (args.count("help") > 0 or argc < 3) {
        std::cout << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(args);
    } catch (po::error& e) {
        vc::Logger()->error(e.what());
        return EXIT_FAILURE;
    }
    // Logging
    auto verbose = args.count("verbose") > 0;

    // Load the pointset
    const fs::path inputPath = args["input"].as<std::string>();
    auto cloud = psio::ReadOrderedPointSet(inputPath);

    // Report the point set properties
    auto cols = cloud.width();
    auto rows = cloud.height();
    auto startZ = static_cast<std::size_t>(cloud[0][2]);
    auto endZ = static_cast<std::size_t>(cloud.getRow(rows - 1)[0][2]);
    vc::Logger()->info(
        "Original pointset :: Shape: ({}, {}), Z-Range: [{}, {}]", rows, cols,
        startZ, endZ);

    // Remap all the rows
    int repaired{0};
    vc::Logger()->info("Repairing pointset...");
    for (const auto row : range(rows)) {
        // get the z-values
        auto newZ = row + startZ;
        auto oldZ = static_cast<std::size_t>(cloud(row, 0)[2]);

        // if the z-values don't match...
        if (oldZ != newZ) {
            // report this row if verbose
            if (verbose) {
                std::cout << "  [" << row << "] ";
                std::cout << oldZ << " -> " << newZ;
                std::cout << "\n";
            }
            // update the row
            for (const auto col : range(cols)) {
                cloud(row, col)[2] = static_cast<double>(newZ);
            }
            // track num repairs
            repaired++;
        }
    }
    vc::Logger()->info("Repaired {} rows", repaired);

    // Report the new point set properties
    cols = cloud.width();
    rows = cloud.height();
    startZ = static_cast<std::size_t>(cloud[0][2]);
    endZ = static_cast<std::size_t>(cloud.getRow(rows - 1)[0][2]);
    vc::Logger()->info(
        "New pointset :: Shape: ({}, {}), Z-Range: [{}, {}]", rows, cols,
        startZ, endZ);

    // (optional) Save the new point cloud
    if (args.count("output") > 0) {
        vc::Logger()->info("Writing pointset...");
        const fs::path outputPath = args["output"].as<std::string>();
        psio::WriteOrderedPointSet(outputPath, cloud);
    }
}