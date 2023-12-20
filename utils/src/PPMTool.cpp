#include <cstddef>
#include <iostream>
#include <regex>
#include <vector>

#include <boost/program_options.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/io/FileExtensionFilter.hpp"
#include "vc/core/io/MeshIO.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/String.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
using namespace volcart;

// Region-of-Interest
struct ROI {
    std::size_t x{0};
    std::size_t y{0};
    std::size_t width{0};
    std::size_t height{0};
};

namespace
{
// Converts ROI string to ROI struct
auto ParseROI(const std::string& opt) -> ROI
{
    // Match against regex
    const std::regex roiRegex{"^[0-9]*x[0-9]*\\+[0-9]*\\+[0-9]*$"};
    if (!std::regex_match(opt.begin(), opt.end(), roiRegex)) {
        Logger()->error("Cannot parse ROI: {}", opt);
        std::exit(EXIT_FAILURE);
    }

    // Split the opt string
    auto strs = split(opt, 'x', ',', '+');
    std::for_each(std::begin(strs), std::end(strs), [](auto& s) { trim(s); });

    if (strs.size() != 4) {
        Logger()->error("Cannot parse ROI: {}", opt);
        std::exit(EXIT_FAILURE);
    }

    return {
        std::stoull(strs[2]), std::stoull(strs[3]), std::stoull(strs[0]),
        std::stoull(strs[1])};
}
}  // namespace

auto main(int argc, char* argv[]) -> int
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("ppm,p", po::value<std::string>()->required(), "Input PPM file")
        ("output-file,o", po::value<std::string>()->required(),
             "Output PPM or mesh file")
        ("roi", po::value<std::string>(), "String describing origin, width, "
             "and height of region-of-interest. Format: WxH+X+Y");

    po::options_description all("Usage");
    all.add(required);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") > 0 || argc < 2) {
        std::cout << all << "\n";
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    // Parse output file

    // Get input file
    const fs::path ppmPath = parsed["ppm"].as<std::string>();
    Logger()->info("Reading PPM...");
    auto ppm = PerPixelMap::ReadPPM(ppmPath);

    const fs::path outPath = parsed["output-mesh"].as<std::string>();
    const auto writeMesh = IsFileType(outPath, {"obj", "ply"});

    // Setup ROI
    size_t minX = 0;
    size_t minY = 0;
    size_t maxX = ppm.width();
    size_t maxY = ppm.height();
    if (parsed.count("roi") > 0) {
        auto roi = ::ParseROI(parsed["roi"].as<std::string>());
        minX = std::max(minX, roi.x);
        minY = std::max(minY, roi.y);
        maxX = std::min(maxX, minX + roi.width);
        maxY = std::min(maxY, minY + roi.height);
    }

    // Convert to an ITKMesh
    if (writeMesh) {
        // Setup output mesh
        auto mesh = ITKMesh::New();

        // Iterate over the ROI
        Logger()->info("Generating point set...");
        ITKPoint pt;
        ITKPixel normal;
        for (auto [y, x] : range2D(minY, maxY, minX, maxX)) {
            // Skip unmapped pixels
            if (!ppm.hasMapping(y, x)) {
                continue;
            }

            const auto id = mesh->GetNumberOfPoints();
            const auto& m = ppm.getMapping(y, x);
            pt[0] = m[0];
            pt[1] = m[1];
            pt[2] = m[2];
            normal[0] = m[3];
            normal[1] = m[4];
            normal[2] = m[5];
            mesh->SetPoint(id, pt);
            mesh->SetPointData(id, normal);
        }

        // Write the mesh
        Logger()->info("Write OBJ file...");
        WriteMesh(outPath, mesh);
    } else {
        Logger()->info("Cropping PPM...");
        const auto h = maxY - minY;
        const auto w = maxX - minX;
        auto outPPM = PerPixelMap::Crop(ppm, minY, minX, h, w);
        Logger()->info("Writing PPM...");
        PerPixelMap::WritePPM(outPath, outPPM);
    }

    Logger()->info("Done.");
}