#include <iostream>

#include <boost/program_options.hpp>
#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/io/MeshIO.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"
#include "vc/meshing/OrientNormals.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vcm = volcart::meshing;

using psio = vc::PointSetIO<cv::Vec3d>;

auto main(int argc, char* argv[]) -> int
{
    ///// Parse the command line options /////
    // clang-format off
    po::options_description all("Usage");
    all.add_options()
        ("help,h", "Show this message")
        ("input-cloud,i", po::value<std::string>()->required(),
            "Path to the input Ordered Point Set")
        ("output-mesh,o", po::value<std::string>()->required(),
            "Path for the output mesh")
        ("mode,m", po::value<int>()->default_value(1),
            "Reading mode: 0 = ASCII, 1 = Binary")
        ("disable-triangulation", "Disable vertex triangulation")
        ("orient-normals", "Auto-orient surface normals towards the mesh centroid");
    // clang-format on

    // parsed will hold the values of all parsed options as a Map
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") > 0 || argc < 2) {
        std::cout << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        vc::Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    fs::path inputPath = parsed["input-cloud"].as<std::string>();
    fs::path outputPath = parsed["output-mesh"].as<std::string>();
    auto mode = static_cast<vc::IOMode>(parsed["mode"].as<int>());

    // Load the file
    vc::Logger()->info("Loading file...");
    auto inputCloud = psio::ReadOrderedPointSet(inputPath, mode);

    // Convert to a mesh
    vc::Logger()->info("Generating mesh...");
    vcm::OrderedPointSetMesher mesher(inputCloud);
    mesher.setComputeTriangulation(parsed.count("disable-triangulation") == 0);
    auto output = mesher.compute();

    // Reorient the surface normals
    if (parsed.count("orient-normals") > 0) {
        vcm::OrientNormals orient;
        orient.setReferenceMode(vcm::OrientNormals::ReferenceMode::Centroid);
        orient.setMesh(output);
        output = orient.compute();
    }

    // Write the mesh
    vc::Logger()->info("Writing mesh...");
    vc::WriteMesh(outputPath, output);
}
