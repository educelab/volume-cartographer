#include <iostream>

#include <boost/program_options.hpp>
#include <itkCompositeTransform.h>
#include <itkTransformFactory.h>
#include <itkTransformFileReader.h>

#include "vc/core/filesystem.hpp"
#include "vc/core/io/ImageIO.hpp"
#include "vc/core/io/MeshIO.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/texturing/ProjectMesh.hpp"

namespace vc = volcart;
namespace fs = vc::filesystem;
namespace vct = vc::texturing;
namespace po = boost::program_options;

using CompositeTransform = itk::CompositeTransform<double, 2>;

auto main(int argc, char* argv[]) -> int
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description general("General Options");
    general.add_options()
        ("help,h", "Show this message")
        ("input-mesh,i", po::value<std::string>()->required(),
            "Input mesh with reordered texture")
        ("output-ppm,o", po::value<std::string>()->required(),
            "Output PPM file")
        ("target-image,t", po::value<std::string>(),
            "Input target (fixed) image")
        ("tfm", po::value<std::string>(),
            "Transform file that maps the texture image onto the target image")
        ("use-first-intersection",
            "Use the ray's first intersection with the mesh rather than the last")
        ("inverse", "If enabled, try to use the inverse transform");

    po::options_description all("Usage");
    all.add(general);
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
        vc::Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    // Get cmd line params
    fs::path meshPath = parsed["input-mesh"].as<std::string>();
    fs::path targetPath;
    if (parsed.count("target-image") > 0) {
        targetPath = parsed["target-image"].as<std::string>();
    }
    fs::path tfmPath;
    if (parsed.count("tfm") > 0) {
        tfmPath = parsed["tfm"].as<std::string>();
    }
    fs::path outPath = parsed["output-ppm"].as<std::string>();
    auto useFirstIntersection = parsed.count("use-first-intersection") > 0;
    auto useInverse = parsed.count("inverse") > 0;

    /** Load mesh **/
    vc::Logger()->info("Loading mesh...");
    auto meshReaderResult = vc::ReadMesh(meshPath);
    auto mesh = meshReaderResult.mesh;
    auto texture = meshReaderResult.texture;

    // Output
    int ppmWidth, ppmHeight;
    if (!targetPath.empty()) {
        auto target = vc::ReadImage(targetPath);
        ppmWidth = target.cols;
        ppmHeight = target.rows;
    } else {
        ppmWidth = texture.cols;
        ppmHeight = texture.rows;
    }

    // Load tfm
    CompositeTransform::Pointer transform;
    if (!tfmPath.empty()) {
        // Register transforms
        itk::TransformFactoryBase::RegisterDefaultTransforms();

        // Read transform
        auto tfmReader = itk::TransformFileReader::New();
        tfmReader->SetFileName(tfmPath.string());
        tfmReader->Update();

        transform = dynamic_cast<CompositeTransform*>(
            tfmReader->GetTransformList()->begin()->GetPointer());
    }

    /** Process mesh **/
    vc::Logger()->info("Projecting mesh...");
    vct::ProjectMesh projector;
    projector.setMesh(mesh);
    projector.setSampleMode(vct::ProjectMesh::SampleMode::Dimensions);
    projector.setTextureDimensions(texture.cols, texture.rows);
    projector.setPPMDimensions(ppmWidth, ppmHeight);
    projector.setTransform(transform);
    projector.setUseInverseTransform(useInverse);
    projector.setUseFirstIntersection(useFirstIntersection);
    auto ppm = projector.compute();

    /** Save PPM **/
    vc::Logger()->info("Saving PPM...");
    vc::PerPixelMap::WritePPM(outPath, ppm);

    return EXIT_SUCCESS;
}
