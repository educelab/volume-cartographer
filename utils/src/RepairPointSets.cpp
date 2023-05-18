#include <algorithm>
#include <numeric>

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
    auto cloud = psio::ReadOrderedPointSet(inputPath);

    // Test for dry run
    auto dryRun = args.count("output") == 0;

    // Report the point set properties
    auto cols = cloud.width();
    auto rows = cloud.height();
    auto startZ = static_cast<std::size_t>(cloud[0][2]);
    auto endZ = static_cast<std::size_t>(cloud.getRow(rows - 1)[0][2]);
    vc::Logger()->info(
        "Pointset loaded :: Shape: ({}, {}), Z-Range: [{}, {}]", rows, cols,
        startZ, endZ);

    // Scan the pointset
    std::vector<std::size_t> mappings(rows);
    for (const auto rIdx : range(rows)) {
        auto row = cloud.getRow(rIdx);
        auto zIdx = static_cast<std::size_t>(row[0][2]);

        mappings[rIdx] = zIdx;
        //        std::cout << "[" << rIdx << "] " << zIdx << "\n";
    }

    // Report errors and remap rows
    auto origMappings = mappings;
    for (const auto row : range(rows)) {
        // z-values
        const auto rZ = mappings[row];

        // Check if the next row is a duplicate
        auto duplicate = (row < rows - 1) ? mappings[row + 1] == rZ : false;

        if (not duplicate) {
            continue;
        }

        // Check where there's a gap in the sequence
        auto gapPrev = (row > 0) ? mappings[row - 1] != rZ - 1 : true;

        auto r = row;
        if (gapPrev) {
            mappings[row] = rZ - 1;
            if (row == 0) {
                startZ -= 1;
            }
        }

        else {
            // TODO: Handle end of sequence
            mappings[row + 1] = rZ + 1;
            r = row + 1;
        }

        std::cout << "[" << r << "] " << rZ << "->" << mappings[r] << "\n";
    }

    // Print new mappings
    for (const auto row : range(rows)) {
        const auto expected = row + startZ;
        const auto actual = mappings[row];

        if (actual != expected) {
            std::cout << "[" << row << "] " << actual << " != " << expected;
            std::cout << "\n";
        }
    }

    // (optional) Save the new point cloud
    if (not dryRun) {
        PointSet outCloud(cols);
        for (const auto r : range(rows)) {
            auto row = cloud.getRow(r);
            auto z = static_cast<double>(mappings[r]);
            std::for_each(row.begin(), row.end(), [z](auto& v) { v[2] = z; });
            outCloud.pushRow(row);
        }

        vc::Logger()->info("Writing pointset...");
        const fs::path outputPath = args["output"].as<std::string>();
        psio::WriteOrderedPointSet(outputPath, outCloud);
    }
}