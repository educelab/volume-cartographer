#include <cmath>
#include <cstdint>

#include <boost/program_options.hpp>

#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/FileFilters.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYReader.hpp"
#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

using psio = vc::PointSetIO<cv::Vec3d>;
using vc::enumerate;

void PointSetToMesh(const fs::path& inputPath, const fs::path& outputPath);
void MeshToPointSet(const fs::path& inputPath, const fs::path& outputPath);

po::variables_map PARSED;

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
        ("output,o", po::value<std::string>()->required(),
             "Path for the output mesh (OBJ/PLY)")
        ("volpkg,v", po::value<std::string>(), "Path to volume package")
        ("volume", po::value<std::string>(),"Sample point intensity from this volume");
    // clang-format on

    // parsed will hold the values of all parsed options as a Map
    po::store(po::command_line_parser(argc, argv).options(all).run(), PARSED);

    // Show the help message
    if (PARSED.count("help") || argc < 2) {
        std::cout << all << '\n';
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(PARSED);
    } catch (po::error& e) {
        vc::Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    fs::path inputPath = PARSED["input"].as<std::string>();
    fs::path outputPath = PARSED["output"].as<std::string>();

    if (vc::IsFileType(inputPath, {"vcps"})) {
        PointSetToMesh(inputPath, outputPath);
    } else if (vc::IsFileType(inputPath, {"obj", "ply"})) {
        MeshToPointSet(inputPath, outputPath);
    } else {
        vc::Logger()->error(
            "Input file is unsupported: {}", inputPath.string());
        return EXIT_FAILURE;
    }
}

void PointSetToMesh(const fs::path& inputPath, const fs::path& outputPath)
{
    // Load the file
    vc::Logger()->info("Loading file...");
    auto inputCloud = psio::ReadPointSet(inputPath);
    vc::Logger()->info("Loaded PointSet with {} points", inputCloud.size());

    // Add vertex intensity
    std::vector<std::uint16_t> intensities;
    if (PARSED.count("volpkg")) {
        // Load the volume package
        auto volpkgPath = PARSED["volpkg"].as<std::string>();
        vc::VolumePkg volpkg(volpkgPath);

        // Load the volume
        vc::Volume::Pointer volume;
        if (PARSED.count("volume")) {
            auto volID = PARSED["volume"].as<std::string>();
            volume = volpkg.volume(volID);
        } else {
            volume = volpkg.volume();
        }

        // Sort cloud by z-index to avoid cache thrashing
        std::sort(
            inputCloud.begin(), inputCloud.end(),
            [](const auto& l, const auto& r) { return l[2] < r[2]; });

        // Get color info
        for (const auto& pt : inputCloud) {
            intensities.emplace_back(volume->interpolateAt(pt));
        }
    }

    // Convert to ITKMesh
    vc::ITKPoint tmpPt;
    auto mesh = vc::ITKMesh::New();
    for (const auto it : enumerate(inputCloud)) {
        tmpPt[0] = it.second[0];
        tmpPt[1] = it.second[1];
        tmpPt[2] = it.second[2];

        mesh->SetPoint(it.first, tmpPt);
    }

    // Write the file
    if (vc::IsFileType(outputPath, {"ply"})) {
        vc::Logger()->info("Writing to PLY...");
        vc::io::PLYWriter writer(outputPath, mesh);
        writer.setVertexColors(intensities);
        writer.write();
        vc::Logger()->info("File written: {}", outputPath.string());
    } else if (vc::IsFileType(outputPath, {"obj"})) {
        vc::Logger()->info("Writing to OBJ...");
        vc::io::OBJWriter writer;
        writer.setPath(outputPath);
        writer.setMesh(mesh);
        writer.write();
        vc::Logger()->info("File written: {}", outputPath.string());
    } else {
        vc::Logger()->error("Unknown file format: {}", outputPath.string());
        std::exit(EXIT_FAILURE);
    }
}

void MeshToPointSet(const fs::path& inputPath, const fs::path& outputPath)
{
    // Load the file
    vc::ITKMesh::Pointer mesh;
    vc::Logger()->info("Loading file...");
    if (vc::IsFileType(inputPath, {"ply"})) {
        vc::io::PLYReader reader(inputPath);
        mesh = reader.read();
    } else if (vc::IsFileType(inputPath, {"obj"})) {
        vc::io::OBJReader reader;
        reader.setPath(inputPath);
        mesh = reader.read();
    }
    vc::Logger()->info("Loaded mesh with {} points", mesh->GetNumberOfPoints());

    // Create pointset
    vc::PointSet<cv::Vec3d> ps;
    for (auto pt = mesh->GetPoints()->Begin(); pt != mesh->GetPoints()->End();
         ++pt) {
        ps.emplace_back(pt->Value()[0], pt->Value()[1], pt->Value()[2]);
    }
    vc::Logger()->info("Writing PointSet...");
    psio::WritePointSet(outputPath, ps);
    vc::Logger()->info("File written: {}", outputPath.string());
}
