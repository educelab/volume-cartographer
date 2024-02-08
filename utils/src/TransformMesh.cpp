#include <iostream>

#include <boost/program_options.hpp>
#include <itkAffineTransform.h>
#include <itkCompositeTransform.h>
#include <itkTransformFileReader.h>
#include <itkTransformFileWriter.h>
#include <itkTransformMeshFilter.h>

#include "vc/core/filesystem.hpp"
#include "vc/core/io/MeshIO.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

// Transforms
using AffineTransform = itk::AffineTransform<double, 3>;
using Displacement = AffineTransform::OutputVectorType;
using CompositeTransform = itk::CompositeTransform<double, 3>;
using MeshTransformer =
    itk::TransformMeshFilter<vc::ITKMesh, vc::ITKMesh, CompositeTransform>;
using TransformWriter = itk::TransformFileWriterTemplate<double>;
using TransformReader = itk::TransformFileReaderTemplate<double>;

auto main(int argc, char** argv) -> int
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
        ("input-tfm", po::value<std::string>(), "Input transformation file")
        ("output-tfm,t", po::value<std::string>(), "Output transformation file");

    po::options_description transformOpts("Transformations");
    transformOpts.add_options()
        ("translate-x,x", po::value<double>()->default_value(0), "X translation")
        ("translate-y,y", po::value<double>()->default_value(0), "Y translation")
        ("translate-z,z", po::value<double>()->default_value(0), "Z translation")
        ("scale,s", po::value<double>()->default_value(1), "Linear scale factor")
        ("scale-precompose", "If enabled, precompose scale transform");

    po::options_description all("Usage");
    all.add(required).add(transformOpts);
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
        std::cerr << "ERROR: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    // Load mesh
    std::cout << "Loading mesh..." << std::endl;
    fs::path inputPath = parsed["input-mesh"].as<std::string>();
    auto meshGroup = vc::ReadMesh(inputPath);
    auto mesh = meshGroup.mesh;

    // Setup composite transform
    auto compositeTrans = CompositeTransform::New();

    // Load transform
    if (parsed.count("input-tfm") > 0) {
        fs::path tfmPath = parsed["input-tfm"].as<std::string>();
        auto readTfm = TransformReader::New();
        readTfm->SetFileName(tfmPath.string());
        readTfm->Update();
        auto it = readTfm->GetTransformList()->begin();
        auto* tfm = static_cast<CompositeTransform*>((*it).GetPointer());
        compositeTrans->AddTransform(tfm);
    }

    // Build affine transform
    else {
        auto affine = AffineTransform::New();

        // Add translation
        Displacement displacement;
        displacement[0] = parsed["translate-x"].as<double>();
        displacement[1] = parsed["translate-y"].as<double>();
        displacement[2] = parsed["translate-z"].as<double>();
        affine->Translate(displacement);

        // Add scale
        auto precompose = parsed.count("scale-precompose") > 0;
        affine->Scale(parsed["scale"].as<double>(), precompose);

        // Build composite transform
        compositeTrans->AddTransform(affine);
    }

    // Simplify the transform
    compositeTrans->FlattenTransformQueue();

    // Apply the composite transform to the mesh
    std::cout << "Applying transform..." << std::endl;
    auto transformer = MeshTransformer::New();
    transformer->SetInput(mesh);
    transformer->SetTransform(compositeTrans);
    transformer->Update();
    auto output = transformer->GetOutput();

    // Write the new mesh
    std::cout << "Writing mesh..." << std::endl;
    fs::path outputPath = parsed["output-mesh"].as<std::string>();
    vc::WriteMesh(outputPath, output, meshGroup.uv, meshGroup.texture);

    ///// Write the final transformations /////
    if (parsed.count("output-tfm") > 0) {
        fs::path transformPath = parsed["output-tfm"].as<std::string>();
        std::cout << "Writing transformation to file..." << std::endl;

        auto transformWriter = TransformWriter::New();
        transformWriter->SetFileName(transformPath.string());
        transformWriter->SetInput(compositeTrans);
        transformWriter->Update();
    }

    return EXIT_SUCCESS;
}
