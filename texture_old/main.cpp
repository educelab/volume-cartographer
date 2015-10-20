// main.cpp
// Chao Du Nov 2014

#include <stdio.h>

#include "CMesh.h"
#include "CPlyHelper.h"
#include "CPoint.h"

#include "volumepkg.h"

#include "modules/meshTexturing.h"

int main( int argc, char *argv[] )
{
    double radius;
    if ( argc < 6 ) {
        std::cout << "Usage: vc_texture volpkg seg-id radius texture-method sample-direction" << std::endl;
        std::cout << "Texture methods: " << std::endl;
        std::cout << "      0 = Intersection" << std::endl;
        std::cout << "      1 = Non-Maximum Suppression" << std::endl;
        std::cout << "      2 = Minimum" << std::endl;
        std::cout << "      3 = Median w/ Averaging" << std::endl;
        std::cout << "      4 = Median w/o Averaging" << std::endl;
        std::cout << "      5 = Mean" << std::endl;
        std::cout << std::endl;
        std::cout << "Sample Direction: " << std::endl;
        std::cout << "      0 = Omni" << std::endl;
        std::cout << "      1 = Positive" << std::endl;
        std::cout << "      2 = Negative" << std::endl;
        exit( -1 );
    }

    VolumePkg volpkg( argv[ 1 ] );
    std::string segID = argv[ 2 ];
    if (segID == "") {
        std::cerr << "ERROR: Incorrect/missing segmentation ID!" << std::endl;
        exit(EXIT_FAILURE);
    }
    if ( volpkg.getVersion() < 2.0) {
        std::cerr << "ERROR: Volume package is version " << volpkg.getVersion() << " but this program requires a version >= 2.0."  << std::endl;
        exit(EXIT_FAILURE);
    }
    volpkg.setActiveSegmentation(segID);

    ChaoVis::CMesh aMesh = volpkg.openMesh();

    radius = atof( argv[ 3 ] );

    int aFindBetterTextureMethod = atoi( argv[ 4 ] );
    EFilterOption aFilterOption = ( EFilterOption )aFindBetterTextureMethod;

    int aSampleDir = atoi( argv[ 5 ] ); // sampleDirection (0=omni, 1=positive, 2=negative)
    EDirectionOption aDirectionOption = ( EDirectionOption )aSampleDir;

    meshTexturing( aMesh, // mesh
                   volpkg, // volume package
                   radius, // radius 1
                   radius / 3.0, // REVISIT - radius 2
                   aFilterOption,
                   aDirectionOption );

    printf( "writing result\n" );
    volpkg.saveTexturedMesh(aMesh);

    return 0;
}
