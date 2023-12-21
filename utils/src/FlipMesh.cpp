#include <iostream>

#include <boost/program_options.hpp>
#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Logging.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

// Volpkg version required by this app
static constexpr int VOLPKG_MIN_VERSION = 6;

int main(int argc, char** argv)
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("volume", po::value<std::string>(), "Volume to use for texturing. "
           "Default: The first volume in the volume package.")
        ("input-mesh,i", po::value<std::string>()->required(),
           "Input mesh file")
        ("output-mesh,o", po::value<std::string>()->required(),
           "Output mesh file")
        ("axis,a", po::value<int>()->default_value(2),
           "Axis along which to flip:\n"
             "  0 = X\n"
             "  1 = Y\n"
             "  2 = Z");

    po::options_description all("Usage");
    all.add(required);
    // clang-format on

    // Parse the cmd line
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
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Get the parsed options
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();

    ///// Load the volume package /////
    auto vpkg = vc::VolumePkg::New(volpkgPath);
    if (vpkg->version() < VOLPKG_MIN_VERSION) {
        vc::Logger()->error(
            "Volume Package is version {} but this program requires version "
            "{}+. ",
            vpkg->version(), VOLPKG_MIN_VERSION);
        return EXIT_FAILURE;
    }

    // Load the Volume
    vc::Volume::Pointer volume;
    try {
        if (parsed.count("volume") > 0) {
            volume = vpkg->volume(parsed["volume"].as<std::string>());
        } else {
            volume = vpkg->volume();
        }
    } catch (const std::exception& e) {
        std::cerr << "Cannot load volume. ";
        std::cerr << "Please check that the Volume Package has volumes and "
                     "that the volume ID is correct."
                  << std::endl;
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Get flipping index
    auto index = parsed["axis"].as<int>();

    // Get the volume bound. Upper bound is not inclusive
    auto volBound = volume->bounds().getUpperBoundByIndex(index) - 1;

    // Load mesh
    fs::path inputPath = parsed["input-mesh"].as<std::string>();
    vc::io::OBJReader reader;
    reader.setPath(inputPath);
    auto mesh = reader.read();

    // Update the mesh
    for (auto pt = mesh->GetPoints()->Begin(); pt != mesh->GetPoints()->End();
         pt++) {
        pt->Value()[index] = volBound - pt->Value()[index];
    }

    // Write the new mesh
    fs::path outputPath = parsed["output-mesh"].as<std::string>();
    vc::io::OBJWriter writer;
    writer.setPath(outputPath);
    writer.setMesh(mesh);
    try {
        writer.setUVMap(reader.getUVMap());
        writer.setTexture(reader.getTextureMat());
    } catch (...) {
        // Do nothing if there's no UV map or Texture image
    }
    writer.write();

    return EXIT_SUCCESS;
}
