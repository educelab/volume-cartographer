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
        ("output,o", po::value<std::string>(), "Path to the output PointSet");
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

    // Load the pointset
    const fs::path inputPath = args["input"].as<std::string>();
    auto inputCloud = psio::ReadOrderedPointSet(inputPath);

    // Test for dry run
    auto dryRun = args.count("output") == 0;

    // Report the point set properties
    auto numCols = inputCloud.width();
    auto numRows = inputCloud.height();
    auto startZ = static_cast<std::size_t>(inputCloud[0][2]);
    auto endZ = static_cast<std::size_t>(inputCloud.getRow(numRows - 1)[0][2]);
    vc::Logger()->info(
        "Pointset loaded :: Shape: ({}, {}), Z-Range: [{}, {}]", numRows,
        numCols, startZ, endZ);

    // Scan the pointset
    std::vector<std::size_t> counts(numRows, 0);
    for (const auto rIdx : range(numRows)) {
        auto row = inputCloud.getRow(rIdx);
        auto cIdx = static_cast<std::size_t>(row[0][2]) - startZ;
        counts.at(cIdx) += row.size();
    }

    // Report missing rows
    for (const auto [cIdx, cnt] : enumerate(counts)) {
        const auto zIdx = cIdx + startZ;
        if (cnt == 0) {
            vc::Logger()->warn("Z={} is empty", zIdx);
        } else if (cnt != numCols) {
            vc::Logger()->warn("Z={} has {} extra points", zIdx, cnt - numCols);
        }
    }

    // (optional) Save the new point cloud
    if (not dryRun) {
        const fs::path outputPath = args["output"].as<std::string>();
    }
}