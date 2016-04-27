//
// Created by Seth Parker on 3/14/16.
//

#include <iostream>

#include <vtkPLYReader.h>
#include <boost/program_options.hpp>

#include "vc_defines.h"
#include "itk2vtk.h"
#include "clothModelingUV.h"
#include "io/objWriter.h"
#include "compositeTextureV2.h"

namespace po = boost::program_options;

void getPins( std::string path, VC_MeshType::Pointer mesh, volcart::texturing::clothModelingUV::PinIDs &pinList );

int main( int argc, char* argv[] ) {

    // Set up options
    // clang-format off
    po::options_description required("Required arguments");
    required.add_options()
            ("help,h", "Show this message")
            ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
            ("input-mesh,i", po::value<std::string>()->required(), "Input mesh path [PLY]")
            ("generate-texture,t",
                    po::value<bool>()->default_value(false),
                    "Generate a textured mesh from the resulting UV map");

    // Unfurl options
    po::options_description unfurlOptions("Unfurl options");
    unfurlOptions.add_options()
            ("unfurl-iterations", po::value<uint16_t>()->required(),
             "Number of iterations to run the unfurl step")
            ("unfurl-a", po::value<double>()->default_value(10),
             "Acceleration rate of unpinned points (m/s^2) during the unfurl step")
            ("unfurl-pins", po::value<std::string>(), "PLY containing pins used during unfurl step");

    // Collision options
    po::options_description collisionOptions("Collision options");
    collisionOptions.add_options()
            ("collision-iterations", po::value<uint16_t>()->required(),
             "Number of iterations to run the collision step")
            ("collision-a", po::value<double>()->default_value(-10),
             "Acceleration rate of unpinned points (m/s^2) during the collision step");

    // Expansion options
    po::options_description expandOptions("Expansion/Relaxation options");
    expandOptions.add_options()
            ("expand-iterations", po::value<uint16_t>()->required(),
             "Number of iterations to run the expansion step")
            ("expand-a", po::value<double>()->default_value(10),
             "Acceleration rate of unpinned points (m/s^2) during the expansion step")
            ("expand-pins", po::value<std::string>(), "PLY containing pins used during expansion step");

    // clang-format on
    po::options_description all("Usage");
    all.add(required).add(unfurlOptions).add(collisionOptions).add(expandOptions);

    // Parse and handle options
    po::variables_map opts;
    po::store(po::parse_command_line(argc, argv, all), opts);

    // Display help
    if (argc == 1 || opts.count("help")) {
        std::cout << all << std::endl;
        std::exit(1);
    }

    // Warn of missing options
    try {
        po::notify(opts);
    } catch (po::error& e) {
        std::cerr << "[error]: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // assign the parsed options
    double unfurlA, collideA, expandA;
    std::string uPins_path, ePins_path;

    VolumePkg vpkg(opts["volpkg"].as<std::string>());
    std::string input_path = opts["input-mesh"].as<std::string>();
    bool genTexture = opts["generate-texture"].as<bool>();

    uint16_t unfurlIt = opts["unfurl-iterations"].as<uint16_t>();
    if ( opts.count("unfurl-a") ) unfurlA = opts["unfurl-a"].as<double>();
    if ( opts.count("unfurl-pins") ) uPins_path = opts["unfurl-pins"].as<std::string>();

    uint16_t collisionIt = opts["collision-iterations"].as<uint16_t>();
    if ( opts.count("collision-a") ) collideA = opts["collision-a"].as<double>();

    uint16_t expansionIt = opts["expansion-iterations"].as<uint16_t>();
    if ( opts.count("expand-a") ) expandA = opts["expand-a"].as<double>();
    if ( opts.count("expand-pins") ) ePins_path = opts["expand-pins"].as<std::string>();

    // Get Mesh
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName ( input_path.c_str() );
    reader->Update();
    VC_MeshType::Pointer mesh = VC_MeshType::New();
    volcart::meshing::vtk2itk(reader->GetOutput(), mesh);

    // Get pinned points for unfurling step
    volcart::texturing::clothModelingUV::PinIDs unfurl;
    getPins( uPins_path, mesh, unfurl);

    // Get pinned points for expansion step
    volcart::texturing::clothModelingUV::PinIDs expand;
    getPins( ePins_path, mesh, expand);

    // Run the simulation
    volcart::texturing::clothModelingUV clothUV( mesh, unfurlIt, collisionIt, expansionIt, unfurl, expand);
    clothUV.setAcceleration( volcart::texturing::clothModelingUV::Stage::Unfurl, 10);
    clothUV.setAcceleration( volcart::texturing::clothModelingUV::Stage::Collision, -10);
    clothUV.setAcceleration( volcart::texturing::clothModelingUV::Stage::Expansion, 10);
    clothUV.run();

    // Write the scaled mesh
    VC_MeshType::Pointer output = clothUV.getMesh();
    std::string path = VC_DATE_TIME() + "_uvMap.obj";
    volcart::io::objWriter writer(path, output);
    writer.write();

    if ( !genTexture ) return EXIT_SUCCESS;

    // Convert soft body to itk mesh
    volcart::UVMap uvMap = clothUV.getUVMap();
    int width, height;

    width = std::ceil( uvMap.ratio().width );
    height = std::ceil( uvMap.ratio().height );

    volcart::texturing::compositeTextureV2 result( mesh, vpkg, clothUV.getUVMap(), 7, width, height);
    volcart::io::objWriter objwriter("textured.obj", mesh, uvMap, result.texture().getImage(0));
    objwriter.write();

    if ( result.texture().getMask().data )
        cv::imwrite("PerPixelMask.png", result.texture().getMask() );

    if ( result.texture().getMap().initialized() ) {
        result.texture().getMap().write( "PerPixelMapping" );
    }

    return 0;
}

/////////// Get pinned points from file //////////
void getPins( std::string path, VC_MeshType::Pointer mesh, volcart::texturing::clothModelingUV::PinIDs &pinList ) {

    // Clear the pin list
    pinList.clear();

    // Load the pin list mesh from file
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName ( path.c_str() );
    reader->Update();
    VC_MeshType::Pointer pins = VC_MeshType::New();
    volcart::meshing::vtk2itk(reader->GetOutput(), pins);

    // Setup points locator
    typename VC_PointsLocatorType::Pointer pointsLocator = VC_PointsLocatorType::New();
    pointsLocator->SetPoints(mesh->GetPoints());
    pointsLocator->Initialize();

    // Iterate over all of the pins and find them in the mesh, add their IDs to the pinList
    for (VC_PointsInMeshIterator pin = pins->GetPoints()->Begin(); pin != pins->GetPoints()->End(); ++pin) {
        unsigned long pinID = pointsLocator->FindClosestPoint( pins->GetPoint( pin->Index() ) );
        pinList.push_back( pinID );
    }

}