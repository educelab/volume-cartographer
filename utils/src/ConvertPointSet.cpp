#include <cmath>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/core/io/FileExtensionFilter.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYReader.hpp"
#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

using psio = vc::PointSetIO<cv::Vec3d>;
using vc::enumerate;

void PointSetToMesh(const fs::path& inputPath, const fs::path& outputPath);
void MeshToPointSet(const fs::path& inputPath, const fs::path& outputPath);

po::variables_map PARSED;

int main(int argc, char* argv[])
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
        std::cout << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(PARSED);
    } catch (po::error& e) {
        vc::logger->error(e.what());
        return EXIT_FAILURE;
    }

    fs::path inputPath = PARSED["input"].as<std::string>();
    fs::path outputPath = PARSED["output"].as<std::string>();

    if (vc::IsFileType(inputPath, {"vcps"})) {
        PointSetToMesh(inputPath, outputPath);
    } else if (vc::IsFileType(inputPath, {"obj", "ply"})) {
        MeshToPointSet(inputPath, outputPath);
    } else {
        vc::logger->error("Input file is unsupported: {}", inputPath.string());
        return EXIT_FAILURE;
    }
}

void PointSetToMesh(const fs::path& inputPath, const fs::path& outputPath)
{
    // Load the file
    vc::logger->info("Loading file...");
    auto inputCloud = psio::ReadPointSet(inputPath);
    vc::logger->info("Loaded PointSet with {} points", inputCloud.size());

    // Add vertex intensity
    std::vector<uint16_t> intensities;
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
        for (const auto& pt :
             vc::ProgressWrap(inputCloud, "Getting point intensities")) {
            intensities.emplace_back(volume->interpolateAt(pt));
        }
    }

    // Convert to ITKMesh
    vc::ITKPoint tmpPt;
    auto mesh = vc::ITKMesh::New();
    for (const auto it :
         vc::ProgressWrap(enumerate(inputCloud), "Converting points")) {
        tmpPt[0] = it.second[0];
        tmpPt[1] = it.second[1];
        tmpPt[2] = it.second[2];

        mesh->SetPoint(it.first, tmpPt);
    }

    // Write the file
    if (vc::IsFileType(outputPath, {"ply"})) {
        vc::logger->info("Writing to PLY...");
        vc::io::PLYWriter writer(outputPath, mesh);
        writer.setVertexColors(intensities);
        writer.write();
        vc::logger->info("File written: {}", outputPath.string());
    } else if (vc::IsFileType(outputPath, {"obj"})) {
        vc::logger->info("Writing to OBJ...");
        vc::io::OBJWriter writer;
        writer.setPath(outputPath);
        writer.setMesh(mesh);
        writer.write();
        vc::logger->info("File written: {}", outputPath.string());
    } else {
        vc::logger->error("Unknown file format: {}", outputPath.string());
        std::exit(EXIT_FAILURE);
    }
}

void MeshToPointSet(const fs::path& inputPath, const fs::path& outputPath)
{
    // Load the file
    vc::ITKMesh::Pointer mesh;
    vc::logger->info("Loading file...");
    if (vc::IsFileType(inputPath, {"ply"})) {
        vc::io::PLYReader reader(inputPath);
        mesh = reader.read();
    } else if (vc::IsFileType(inputPath, {"obj"})) {
        vc::io::OBJReader reader;
        reader.setPath(inputPath);
        mesh = reader.read();
    }
    vc::logger->info("Loaded mesh with {} points", mesh->GetNumberOfPoints());

    // Create pointset
    vc::PointSet<cv::Vec3d> ps;
    for (auto pt = mesh->GetPoints()->Begin(); pt != mesh->GetPoints()->End();
         ++pt) {
        ps.emplace_back(pt->Value()[0], pt->Value()[1], pt->Value()[2]);
    }
    vc::logger->info("Writing PointSet...");
    psio::WritePointSet(outputPath, ps);
    vc::logger->info("File written: {}", outputPath.string());
}
