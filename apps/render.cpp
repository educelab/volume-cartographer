// render.cpp
// Abigail Coleman Feb. 2015

#include <iostream>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "volumepkg.h"

#include "io/ply2itk.h"
#include "io/plyWriter.h"
#include "io/objWriter.h"
#include "compositeTexture.h"
#include "smoothNormals.h"

int main(int argc, char* argv[])
{
    std::cout << "vc_render" << std::endl;
    ///// Parse the command line options /////
    boost::filesystem::path volpkgPath, outputPath;
    std::string segID;
    double radius;
    double smoothRadius = 0;
    VC_Composite_Option aFilterOption;
    VC_Direction_Option aDirectionOption;

    try {
        // All command line options
          boost::program_options::options_description options("Options");
          options.add_options()
                  ("help,h", "Show this message")
                  ("volpkg,v", boost::program_options::value<std::string>()->required(), "Path to the volume package")
                  ("seg,s",    boost::program_options::value<std::string>()->required(), "Segmenation ID number")
                  ("radius,r", boost::program_options::value<int>()->required(), "Texture search radius")
                  ("method,m", boost::program_options::value<int>()->default_value(1), "Texture method:\n"
                                                                   "  0 = Intersection\n"
                                                                   "  1 = Non-Maximum Suppression\n"
                                                                   "  2 = Maximum\n"
                                                                   "  3 = Minimum\n"
                                                                   "  4 = Median w/ Averaging\n"
                                                                   "  5 = Median\n"
                                                                   "  6 = Mean\n")
                  ("direction,d", boost::program_options::value<int>()->default_value(0), "Sample Direction:\n"
                                                                   "  0 = Omni\n"
                                                                   "  1 = Positive\n"
                                                                   "  2 = Negative\n")
                  ("smooth-normals", boost::program_options::value<double>(), "Average the surface normals using points within radius [arg]")
                  ("output-file,o", boost::program_options::value<std::string>(), "Output file path. If not specified, file will be saved to volume package.");

        // parsedOptions will hold the values of all parsed options as a Map
        boost::program_options::variables_map parsedOptions;
        boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
                                              .options(options)
                                              .run(),
                                      parsedOptions);

        // Show the help message
        if (parsedOptions.count("help") || argc < 2) {
            std::cout << options << std::endl;
            return EXIT_SUCCESS;
        }
        // Warn of missing options
        boost::program_options::notify(parsedOptions);

        // Get the parsed options
        volpkgPath = parsedOptions["volpkg"].as<std::string>();
        segID = parsedOptions["seg"].as<std::string>();
        radius = parsedOptions["radius"].as<int>();
        aFilterOption = (VC_Composite_Option) parsedOptions["method"].as<int>();
        aDirectionOption = (VC_Direction_Option) parsedOptions["direction"].as<int>();

        // Check for output file
        if ( parsedOptions.count("output-file") ) {
            outputPath = parsedOptions["output-file"].as<std::string>();
            if ( boost::filesystem::exists(boost::filesystem::canonical(outputPath.parent_path())) )
                outputPath = boost::filesystem::canonical(outputPath.parent_path()).string() + "/" + outputPath.filename().string();
            else
                std::cerr << "ERROR: Cannot write to provided output file. Output directory does not exist." << std::endl;
        }

        // Check for other options
        if ( parsedOptions.count("smooth-normals") )
            smoothRadius =  parsedOptions["smooth-normals"].as<double>();

    }
    catch(std::exception& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    ///// Load the volume package /////
    if ( boost::filesystem::exists(volpkgPath) || (boost::filesystem::canonical(volpkgPath).extension() != ".volpkg" ) ) {
        volpkgPath = boost::filesystem::canonical(volpkgPath);
    } else {
        std::cerr << "ERROR: Volume package does not exist/not recognized at provided path: " << volpkgPath << std::endl;
        return EXIT_FAILURE;
    }

    VolumePkg vpkg( volpkgPath.string() );
    if ( vpkg.getVersion() < 2.0) {
        std::cerr << "ERROR: Volume package is version " << vpkg.getVersion() << " but this program requires a version >= 2.0."  << std::endl;
        return EXIT_FAILURE;
    }
    vpkg.volume().setCacheMemoryInBytes(systemMemorySize());

    ///// Set the segmentation ID /////
    vpkg.setActiveSegmentation(segID);
    std::string meshName = vpkg.getMeshPath();
    
    // declare pointer to new Mesh object
    VC_MeshType::Pointer  input = VC_MeshType::New();
    VC_MeshType::Pointer  workingMesh = VC_MeshType::New();

    int meshWidth = -1;
    int meshHeight = -1;

    // try to convert the ply to an ITK mesh
    if (!volcart::io::ply2itkmesh(meshName, input, meshWidth, meshHeight)){
        exit( -1 );
    };

    // Smooth surface normals
    if ( smoothRadius > 0 ) {
        workingMesh = volcart::meshing::smoothNormals(input, smoothRadius);
    } else {
      // duplicate input
      // To-Do: Should be a deep copy
      workingMesh = input;
    }


    volcart::Texture newTexture;
    newTexture = volcart::texturing::compositeTexture( workingMesh, vpkg, meshWidth, meshHeight, radius, aFilterOption, aDirectionOption );

    if ( outputPath.extension() == ".PLY" || outputPath.extension() == ".ply" ) {
        std::cout << "Writing to PLY..." << std::endl;
        volcart::io::plyWriter writer(outputPath.string(), input, newTexture);
        writer.write();
    } else if ( outputPath.extension() == ".OBJ" || outputPath.extension() == ".obj") {
        std::cout << "Writing to OBJ..." << std::endl;
        volcart::io::objWriter writer(outputPath.string(), input, newTexture.uvMap(), newTexture.getImage(0) );
        writer.write();
    } else if ( outputPath.extension() == ".PNG" || outputPath.extension() == ".png") {
        std::cout << "Writing to PNG..." << std::endl;
        cv::imwrite( outputPath.string(), newTexture.getImage(0) );
    } else {
        std::cout << "Writing to Volume Package..." << std::endl;
        vpkg.saveMesh(input, newTexture);
    }

    return EXIT_SUCCESS;
} // end main

