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
        std::cout << "Usage: sliceIntersection mesh.ply radius wantBetterTexture(0=no, 1=non-maximum suppression, 2=min, 3=median filter, 4=median (without averaging), 5=mean) volPkgPath sampleDirection(0=omni, 1=positive, 2=negative)" << std::endl;
        exit( -1 );
    }

    radius = atof( argv[ 2 ] );

    // (1) read in ply mesh file
    ChaoVis::CMesh aMesh;
    std::string aFileName( argv[ 1 ] );
    ChaoVis::CPlyHelper::ReadPlyFile( aFileName, aMesh );
    std::cout << "Mesh file loaded" << std::endl;
    // REVISIT - for debug
    aMesh.Dump();

    //set output name
    std::string outputName = aFileName.substr(aFileName.find_last_of("/\\")+1);
    outputName = outputName.substr(0,outputName.find_last_of("."));


    // REVISIT - refactored to module
    VolumePkg vpkg = VolumePkg( argv[ 4 ] );

    int aFindBetterTextureMethod = atoi( argv[ 3 ] );
    EFilterOption aFilterOption = ( EFilterOption )aFindBetterTextureMethod;

    int aSampleDir = atoi( argv[ 5 ] ); // sampleDirection (0=omni, 1=positive, 2=negative)
    EDirectionOption aDirectionOption = ( EDirectionOption )aSampleDir;

    meshTexturing( aMesh, // mesh
                   vpkg, // volume package
                   radius, // radius 1
                   radius / 3.0, // REVISIT - radius 2
                   aFilterOption,
                   aDirectionOption );

    printf( "writing result\n" );
    ChaoVis::CPlyHelper::WritePlyFile( outputName + "_textured.ply", aMesh );

    return 0;
}
