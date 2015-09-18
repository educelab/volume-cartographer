/*  9/14/15

Purpose:
    Create a unit test template for VC project.
    This particular test evaluates objExporterExample.cpp.

Test Location:
        Propose having all tests located in {source_dir}/testing folder.
        This way of organizing unit tests allows dev team quick access to the testing structure.
        As an example, this objExporterTest.cpp file is housed in /examples/testing.

        Ideally, all tests could be combined into a suite that could be run and then user could select
        which aspects of the code base to test with either default or user-provided data. We can discuss
        what this could look like in the future.

Usage:

    Test expects a mesh, texture, and uvMap to test objectExporter

    *** Not sure if we need all three together or if we need the mesh OR a texture/uvMap combo ***
    *** argv below accounts only for all three to be included at runtime **


Expected Inputs:

        ** Data Set Needed **
        1. ITK Mesh - default (?)
        2. Texture Image - default (?)
        3. uvMap - default (?)


Expect Outputs:
    1. Mesh object created

    If the input data set does not satisfy the tests, error message output to stdout

    ** Output Evaluation **



*/
#include "vc_defines.h"
#include "io/objWriter.h"
#include "io/ply2itk.h"
#include "testing/testingMesh.h"

int main(int argc, char *argv[]){


    /* Only working with a mesh right now*/
    /*if (argc < 2){
        std::cout << "Need to at least include an inputMesh file" << std::endl
                  << "Usage: ./executable meshFile textureImage uvMap" << std::endl
                  << "Aborting." << std::endl;
        return EXIT_FAILURE;
    }*/

    cv::Mat uvImg = cv::imread( argv[2], CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );
    std::cout << "Creating mesh..." << std::endl;

    volcart::testing::testingMesh  mesh;
    VC_MeshType::Pointer inputMesh = mesh.itkMesh();

    volcart::io::objWriter mesh_writer;
    mesh_writer.setPath( "nothing" );
    //mesh_writer.setUVMap( uvMap );
    mesh_writer.setTexture( uvImg );

    // mesh_writer.write() runs validate() as well, but this lets us handle a mesh that can't be validated.
    if ( mesh_writer.validate() )
        mesh_writer.write();
    else {
        mesh_writer.setPath( "output.obj" );
        mesh_writer.setMesh( inputMesh );
        mesh_writer.write();
    }

    mesh_writer.
    return EXIT_SUCCESS;
}

