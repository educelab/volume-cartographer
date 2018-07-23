#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <itkAffineTransform.h>
#include <itkCompositeTransform.h>
#include <itkTransformFileWriter.h>
#include <itkTransformMeshFilter.h>

#include "vc/core/io/OBJReader.hpp"
#include "vc/core/io/OBJWriter.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

// Transforms
using AffineTransform = itk::AffineTransform<double, 3>;
using Displacement = AffineTransform::OutputVectorType;
using CompositeTransform = itk::CompositeTransform<double, 3>;
using MeshTransformer =
    itk::TransformMeshFilter<vc::ITKMesh, vc::ITKMesh, CompositeTransform>;
using TransformWriter = itk::TransformFileWriterTemplate<double>;

int main(int argc, char** argv)
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
        ("output-tfm,t", po::value<std::string>(), "Output transformation file")
        ("translate-x,x", po::value<double>()->default_value(0), "X translation")
        ("translate-y,y", po::value<double>()->default_value(0), "Y translation")
        ("translate-z,z", po::value<double>()->default_value(0), "Z translation");

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

    // Load mesh
    std::cout << "Loading mesh..." << std::endl;
    fs::path inputPath = parsed["input-mesh"].as<std::string>();
    vc::io::OBJReader reader;
    reader.setPath(inputPath);
    auto mesh = reader.read();

    // Setup composite transform
    auto compositeTrans = CompositeTransform::New();

    // Build affine transform
    auto translate = AffineTransform::New();
    Displacement displacement;
    displacement[0] = parsed["translate-x"].as<double>();
    displacement[1] = parsed["translate-y"].as<double>();
    displacement[2] = parsed["translate-z"].as<double>();
    translate->Translate(displacement);
    compositeTrans->AddTransform(translate);

    // Apply the composite transform to the mesh
    std::cout << "Applying transform..." << std::endl;
    auto output = vc::ITKMesh::New();
    auto transformer = MeshTransformer::New();
    transformer->SetInput(mesh);
    transformer->SetOutput(output);
    transformer->SetTransform(compositeTrans);
    transformer->Update();

    // Write the new mesh
    std::cout << "Writing mesh..." << std::endl;
    fs::path outputPath = parsed["output-mesh"].as<std::string>();
    vc::io::OBJWriter writer;
    writer.setPath(outputPath);
    writer.setMesh(output);
    try {
        writer.setUVMap(reader.getUVMap());
        writer.setTexture(reader.getTextureMat());
    } catch (...) {
        // Do nothing if there's no UV map or Texture image
    }
    writer.write();

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
