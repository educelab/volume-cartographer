#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "vc/core/io/FileExtensionFilter.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYReader.hpp"
#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/util/Logging.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

using psio = vc::PointSetIO<cv::Vec3d>;

void PointSetToMesh(const fs::path& inputPath, const fs::path& outputPath);
void MeshToPointSet(const fs::path& inputPath, const fs::path& outputPath);

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
             "Path for the output mesh (OBJ/PLY)");
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
        vc::logger->error(e.what());
        return EXIT_FAILURE;
    }

    fs::path inputPath = parsed["input"].as<std::string>();
    fs::path outputPath = parsed["output"].as<std::string>();

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

    // Convert to ITKMesh
    auto mesh = vc::ITKMesh::New();

    // Transfer the vertex info
    vc::ITKPoint tmpPt;
    size_t cnt = 0;
    for (auto& i : inputCloud) {
        tmpPt[0] = i[0];
        tmpPt[1] = i[1];
        tmpPt[2] = i[2];

        mesh->SetPoint(cnt, tmpPt);
        ++cnt;
    }

    // Write the file
    if (vc::IsFileType(outputPath, {"ply"})) {
        vc::logger->info("Writing to PLY...");
        vc::io::PLYWriter writer(outputPath, mesh);
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