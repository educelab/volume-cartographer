#include <iostream>
#include <regex>
#include <vector>

#include <boost/program_options.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/io/MeshIO.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/util/String.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

// Region-of-Interest
struct ROI {
    size_t x{0};
    size_t y{0};
    size_t width{0};
    size_t height{0};
};

// Converts ROI string to ROI struct
auto ParseROI(const std::string& opt) -> ROI;

auto main(int argc, char* argv[]) -> int
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("ppm,p", po::value<std::string>()->required(), "Input PPM file")
        ("output-mesh,o", po::value<std::string>()->required(),
             "Output mesh file")
        ("roi", po::value<std::string>(), "String describing origin and width "
             "and height of region-of-interest. Format: WxH+X+Y");

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

    // Get input file
    fs::path ppmPath = parsed["ppm"].as<std::string>();
    std::cout << "Reading PPM..." << std::endl;
    auto ppm = vc::PerPixelMap::ReadPPM(ppmPath);

    // Setup ROI
    size_t minX = 0;
    size_t minY = 0;
    size_t maxX = ppm.width();
    size_t maxY = ppm.height();

    if (parsed.count("roi") > 0) {
        auto roi = ParseROI(parsed["roi"].as<std::string>());
        minX = std::max(minX, roi.x);
        minY = std::max(minY, roi.y);
        maxX = std::min(maxX, minX + roi.width);
        maxY = std::min(maxY, minY + roi.height);
    }

    // Setup output mesh
    auto mesh = vc::ITKMesh::New();

    // Iterate over the ROI
    std::cout << "Generating point set..." << std::endl;
    for (auto y = minY; y < maxY; y++) {
        for (auto x = minX; x < maxX; x++) {
            // Skip unmapped pixels
            if (!ppm.hasMapping(y, x)) {
                continue;
            }

            auto id = mesh->GetNumberOfPoints();
            auto pt = ppm.getAsPixelMap(y, x);
            mesh->SetPoint(id, pt.pos.val);
            mesh->SetPointData(id, pt.normal.val);
        }
    }

    // Write the mesh
    std::cout << "Write OBJ file..." << std::endl;
    fs::path outputPath = parsed["output-mesh"].as<std::string>();
    vc::WriteMesh(outputPath, mesh);
}

// Convert ROI string to ROI struct
auto ParseROI(const std::string& opt) -> ROI
{
    // Match against regex
    std::regex roiRegex{"^[0-9]*x[0-9]*\\+[0-9]*\\+[0-9]*$"};
    if (!std::regex_match(opt.begin(), opt.end(), roiRegex)) {
        std::cerr << "Cannot parse ROI: " << opt << std::endl;
        exit(EXIT_FAILURE);
    }

    // Split the opt string
    auto strs = vc::split(opt, 'x', ',', '+');
    std::for_each(
        std::begin(strs), std::end(strs), [](auto& s) { vc::trim(s); });

    if (strs.size() != 4) {
        std::cerr << "Cannot parse ROI: " << opt << std::endl;
        exit(EXIT_FAILURE);
    }

    return {
        std::stoull(strs[2]), std::stoull(strs[3]), std::stoull(strs[0]),
        std::stoull(strs[1])};
}