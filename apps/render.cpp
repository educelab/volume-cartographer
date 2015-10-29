// main.cpp
// Abigail Coleman Feb. 2015

#include <iostream>
#include <fstream>

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "volumepkg.h"

#include "io/ply2itk.h"
#include "compositeTexture.h"

int main(int argc, char* argv[])
{
    if ( argc < 6 ) {
        std::cout << "Usage: vc_render volpkg seg-id radius texture-method sample-direction" << std::endl;
        std::cout << "Texture methods: " << std::endl;
        std::cout << "      0 = Intersection" << std::endl;
        std::cout << "      1 = Non-Maximum Suppression" << std::endl;
        std::cout << "      2 = Maximum" << std::endl;
        std::cout << "      3 = Minimum" << std::endl;
        std::cout << "      4 = Median w/ Averaging" << std::endl;
        std::cout << "      5 = Median" << std::endl;
        std::cout << "      6 = Mean" << std::endl;
        std::cout << std::endl;
        std::cout << "Sample Direction: " << std::endl;
        std::cout << "      0 = Omni" << std::endl;
        std::cout << "      1 = Positive" << std::endl;
        std::cout << "      2 = Negative" << std::endl;
        exit( -1 );
    }

    VolumePkg vpkg( argv[ 1 ] );
    vpkg.setCacheMemory(systemMemorySize());

    std::string segID = argv[ 2 ];
    if (segID == "") {
        std::cerr << "ERROR: Incorrect/missing segmentation ID!" << std::endl;
        exit(EXIT_FAILURE);
    }
    if ( vpkg.getVersion() < 2.0) {
        std::cerr << "ERROR: Volume package is version " << vpkg.getVersion() << " but this program requires a version >= 2.0."  << std::endl;
        exit(EXIT_FAILURE);
    }
    vpkg.setActiveSegmentation(segID);
    std::string meshName = vpkg.getMeshPath();

    double radius = atof( argv[3] );

    int aFindBetterTextureMethod = atoi( argv[ 4 ] );
    VC_Composite_Option aFilterOption = ( VC_Composite_Option )aFindBetterTextureMethod;

    int aSampleDir = atoi( argv[ 5 ] ); // sampleDirection (0=omni, 1=positive, 2=negative)
    VC_Direction_Option aDirectionOption = ( VC_Direction_Option )aSampleDir;
    
    // declare pointer to new Mesh object
    VC_MeshType::Pointer  mesh = VC_MeshType::New();

    int meshWidth = -1;
    int meshHeight = -1;

    // try to convert the ply to an ITK mesh
    if (!volcart::io::ply2itkmesh(meshName, mesh, meshWidth, meshHeight)){
        exit( -1 );
    };

    volcart::Texture newTexture;
    newTexture = volcart::texturing::compositeTexture( mesh, vpkg, meshWidth, meshHeight, radius, aFilterOption, aDirectionOption );

    vpkg.saveMesh(mesh, newTexture);

    return 0;
} // end main

