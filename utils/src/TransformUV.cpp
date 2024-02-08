#include <iostream>

#include <boost/program_options.hpp>
#include <opencv2/core.hpp>

#include "vc/app_support/GeneralOptions.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/MeshIO.hpp"
#include "vc/core/util/FloatComparison.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/meshing/CalculateNormals.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
using namespace volcart;

template <typename T>
constexpr T PI =
    T(3.141592653589793238462643383279502884198716939937510582097164L);

template <typename T>
constexpr T DEG_TO_RAD = PI<T> / T(180);

auto main(int argc, char* argv[]) -> int
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("input-mesh,i", po::value<std::string>()->required(),
           "Input mesh file")
        ("output-mesh,o", po::value<std::string>()->required(),
           "Output mesh file")
        ("rotate,r", po::value<double>(), "Rotate the UV map "
          "by an angle in degrees (counterclockwise).")
        ("flip,f", po::value<int>(),
            "Flip the UV map along an axis. If rotate is specified, flip is "
           "performed after rotation.\n"
           "Axis along which to flip:\n"
             "  0 = Vertical\n"
             "  1 = Horizontal\n"
             "  2 = Both")
        ("disable-image-transform", "By default, transforms applied to the UV "
           "map are also applied to the texture image. If provided, don't "
           "transform the texture image.")
        ("disable-normal-generation", "By default, per-vertex surface normals "
           "are generated if the mesh doesn't already have normals. If "
           "provided, this normal generation behavior is disabled.");

    po::options_description all("Usage");
    all.add(required).add(GetMeshIOOpts());
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") > 0 || argc < 2) {
        std::cout << all << '\n';
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    // Check for required option
    if (parsed.count("rotate") == 0 and parsed.count("flip") == 0) {
        Logger()->error("Missing one of required options --rotate/--flip");
        return EXIT_FAILURE;
    }

    // Load the mesh
    Logger()->info("Loading input mesh...");
    fs::path inputPath = parsed["input-mesh"].as<std::string>();
    auto inputMesh = ReadMesh(inputPath);

    // Copy UV map and texture
    auto uvMap = inputMesh.uv;
    auto texture = inputMesh.texture;

    // Exit if we don't have a UV map
    if (not uvMap) {
        Logger()->error("Input mesh does not have UV map.");
        return EXIT_FAILURE;
    }

    // Check for image processing
    auto processImage = parsed.count("disable-image-transform") == 0;
    if (processImage and texture.empty()) {
        Logger()->warn(
            "Requested texture image transformation but no "
            "texture image loaded.");
        processImage = false;
    }

    // Rotate
    if (parsed.count("rotate") > 0) {
        // Get the rotation angle
        auto theta = parsed["rotate"].as<double>();
        Logger()->info("Rotating by {} degrees...", theta);

        // Special cases: 90/180/270 deg
        if (theta == 90. or theta == 180. or theta == 270.) {
            // Convert rotation angle to rotation code
            // Subtract from 2 because rotation codes are clockwise ordered
            auto rotateCode = int(2 - (theta - 90.) / 90.);

            // Apply rotation
            if (processImage) {
                UVMap::Rotate(
                    *uvMap, static_cast<UVMap::Rotation>(rotateCode), texture);
            } else {
                UVMap::Rotate(*uvMap, static_cast<UVMap::Rotation>(rotateCode));
            }
        }

        // All other angles
        else if (not AlmostEqual(theta, 0.0)) {
            if (processImage) {
                UVMap::Rotate(*uvMap, theta * DEG_TO_RAD<double>, texture);
            } else {
                UVMap::Rotate(*uvMap, theta * DEG_TO_RAD<double>);
            }
        }
    }

    // Flip
    if (parsed.count("flip") > 0) {
        // Get flipping direction
        auto flipCode = parsed["flip"].as<int>();
        auto axis = static_cast<UVMap::FlipAxis>(flipCode);
        if (flipCode == 2) {
            flipCode = -1;
        }

        // Update the UV map and image
        Logger()->info("Flipping...");
        UVMap::Flip(*uvMap, axis);
        if (processImage) {
            cv::flip(texture, texture, flipCode);
        }
    }

    // Compute mesh normals
    auto mesh = inputMesh.mesh;
    auto addNormals = parsed.count("disable-normal-generation") == 0;
    if (addNormals and mesh->GetPointData()->empty()) {
        Logger()->info("Computing surface normals...");
        meshing::CalculateNormals normalGen(mesh);
        mesh = normalGen.compute();
    }

    // Setup mesh writer opts
    MeshWriterOpts opts;
    if (parsed.count("texture-format") > 0) {
        opts.imgFmt = parsed["texture-format"].as<std::string>();
    }

    // Write the new mesh
    Logger()->info("Writing output mesh...");
    fs::path outputPath = parsed["output-mesh"].as<std::string>();
    WriteMesh(outputPath, mesh, uvMap, texture, opts);

    Logger()->info("Done.");
    return EXIT_SUCCESS;
}
