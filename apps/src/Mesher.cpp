#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/core.hpp>

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

using psio = vc::PointSetIO<cv::Vec3d>;

int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description all("Usage");
    all.add_options()
            ("help,h", "Show this message")
            ("input-cloud,i", po::value<std::string>()->required(),
             "Path to the input Ordered Point Set")
            ("output-mesh,o", po::value<std::string>()->required(),
             "Path for the output mesh")
            ("disable-triangulation", "Disable vertex triangulation");
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
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    fs::path inputPath = parsed["input-cloud"].as<std::string>();
    fs::path outputPath = parsed["output-mesh"].as<std::string>();

    // Load the file
    std::cout << "Loading file..." << std::endl;
    auto inputCloud = psio::ReadOrderedPointSet(inputPath);

    // Convert to a mesh
    std::cout << "Generating mesh..." << std::endl;
    vc::meshing::OrderedPointSetMesher mesher(inputCloud);
    mesher.setComputeTriangulation(parsed.count("disable-triangulation") == 0);
    auto output = mesher.compute();

    // Write the mesh
    std::cout << "Writing OBJ..." << std::endl;
    vc::io::OBJWriter writer;
    writer.setPath(outputPath);
    writer.setMesh(output);
    writer.write();
}
