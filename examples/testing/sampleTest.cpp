//
// Created by Ryan Taber on 9/25/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE sample
#include <boost/test/unit_test.hpp>
#include <boost/test/test_tools.hpp>
#include "testing/testingMesh.h"
#include <vc_defines.h>
#include "io/objWriter.h"
#include "io/ply2itk.h"


//TODO: confirm includes above...probably don't need all these 

//helper function to split lines read in from obj file
std::vector<std::string> split_string(std::string);

/*
 * This builds objects for each of the test cases below that reference
 * the fixture as their second argument
 *
 */

struct meshFix {

    meshFix() {

        //create the mesh for all the test cases to use
        _mesh = mesh.itkMesh();

        BOOST_TEST_MESSAGE("setting up mesh");
    }

    ~meshFix(){ BOOST_TEST_MESSAGE("cleaning up meshFix"); }

    //file path and objwriter to be used in cases
    volcart::io::objWriter mesh_writer;
    boost::filesystem::path objPath;
    VC_MeshType::Pointer _mesh ;
    volcart::testing::testingMesh mesh;
};


// Test for checking successful write
BOOST_FIXTURE_TEST_CASE(writeTest, meshFix) {

        std::cout << "Writing object" << std::endl;

        mesh_writer.setPath("nothing");
        //mesh_writer.setUVMap( uvMap );
        // mesh_writer.setTexture( uvImg );

        objPath = mesh_writer.getPath();
        objPath = boost::filesystem::absolute(objPath);
        std::cout << "File path of mesh obj: " << objPath << std::endl;


        // mesh_writer.write() runs validate() as well, but this lets us handle a mesh that can't be validated.
        if (mesh_writer.validate())
            mesh_writer.write();
        else {
            mesh_writer.setPath("output.obj");
            mesh_writer.setMesh(_mesh);
            mesh_writer.write();
        }

        //check the file path from the mesh_writer.write() call above
        //compare() returns 0 only if paths are same value lexicographically-speaking
        //checking "output.obj" here because the mesh_writer shouldn't validate in the current case
        BOOST_CHECK_EQUAL(mesh_writer.getPath().compare("output.obj"), 0);

}


BOOST_FIXTURE_TEST_CASE(compareElements, meshFix){


    std::vector<VC_Vertex> testPoints = mesh.getPoints();
    std::vector<VC_Cell> testCells = mesh.getCells();

    //   Get absolute path of obj file   //

    objPath = mesh_writer.getPath();
    objPath = boost::filesystem::absolute(objPath);
    std::cout << "File path of mesh obj: " << objPath << std::endl;


    //   Now to get the points and cells from the saved obj file   //

    std::ifstream inputMesh;
    inputMesh.open(objPath.string() + "/output.obj");     //probably better way of assigning filename

    //   Test to see if output.obj is open     //

    if (!inputMesh.is_open()) { BOOST_CHECK(false);}

    //   Declare string to hold lines of obj file   //
    std::string line;
    getline(inputMesh,line);

    //   Vector will hold pieces of line delimited by space (filled by split_string()
    std::vector<std::string> objLine;

    //   Declare vectors to hold mesh points and cells from obj file
    std::vector<VC_Vertex> objPoints;
    std::vector<VC_Cell> objCells;

    //   VC types to put store appropriate values read in from file
    VC_Vertex objVertex ;
    VC_Cell objCell;

    //loop through file and get the appropriate data into the vectors
    while ( !inputMesh.eof() ) {

        objLine = split_string(line);

        //skip comment lines
        if (objLine[0] == "#"){
            getline(inputMesh, line);
            continue;
        }

        //   Vertex
        else if (objLine[0] == "v" && objLine[1] != "n"){
            objVertex.x = std::stod(objLine[1]);
            objVertex.y = std::stod(objLine[2]);
            objVertex.z = std::stod(objLine[3]);
        }


        //Normal
        else if (objLine[0] == "vn"){
            objVertex.nx = std::stod(objLine[1]);
            objVertex.ny = std::stod(objLine[2]);
            objVertex.nz = std::stod(objLine[3]);

            //push vertex onto objPoints
            objPoints.push_back(objVertex);
        }

        //Face
        else if (objLine[0] == "f"){

            // obj file format of 'v/vt/vn' 'v/vt/vn' 'v/vt/vn'
            // seems we should be able to just get the 'v' from each of these sets and subtract one
            // before assigning to v1,v2,v3 for each cell.


            for (size_t faceVertex = 1; faceVertex < 4; faceVertex++ ) {

                size_t lpos = 0;
                size_t pos;

                pos = objLine[faceVertex].find("//", lpos);

                // assign the v1,v2,v3 values...subtract 1 from each of the values to account
                // for off by one error
                if (faceVertex == 1)
                    objCell.v1 = std::stoul(objLine[faceVertex].substr(lpos, pos - lpos));
                else if (faceVertex == 2)
                    objCell.v2 = std::stoul(objLine[faceVertex].substr(lpos, pos - lpos));
                else
                    objCell.v3 = std::stoul(objLine[faceVertex].substr(lpos, pos - lpos));
            }
            //push the cell onto objCells vector
            objCells.push_back(objCell);
        }

        line.clear();
        getline( inputMesh, line );
    }

    // Now to test the objPoints created during fixture init vs points read in from file.
    for (size_t testPoint = 0; testPoint <= sizeof(testPoints); testPoint++){

        BOOST_MESSAGE("Checking Point " << testPoint);

        //checking the x,y,z,nx,ny,nz components

        BOOST_CHECK_EQUAL(testPoints[testPoint].x, objPoints[testPoint].x);
        BOOST_CHECK_EQUAL(testPoints[testPoint].y, objPoints[testPoint].y);
        BOOST_CHECK_EQUAL(testPoints[testPoint].z, objPoints[testPoint].z);

        BOOST_CHECK_EQUAL(testPoints[testPoint].nx, objPoints[testPoint].nx);
        BOOST_CHECK_EQUAL(testPoints[testPoint].ny, objPoints[testPoint].ny);
        BOOST_CHECK_EQUAL(testPoints[testPoint].nz, objPoints[testPoint].nz);

    }

    // Now to test the objPoints created during fixture init vs points read in from file.
    for (size_t testCell = 0; testCell <= sizeof(testCells); testCell++){

        BOOST_MESSAGE("Checking Cell: " << testCell);

        //checking the v1,v2,v3 components

        BOOST_CHECK_EQUAL(testCells[testCell].v1, objCells[testCell].v1 - 1);
        BOOST_CHECK_EQUAL(testCells[testCell].v2, objCells[testCell].v2 - 1);
        BOOST_CHECK_EQUAL(testCells[testCell].v3, objCells[testCell].v3 - 1);

    }

}

std::vector<std::string> split_string(std::string input)
{
    std::vector<std::string> line;

    size_t field_start = 0;
    size_t next_space;

    do{
        next_space = input.find(' ', field_start);

        if (next_space == std::string::npos){
            line.push_back(input.substr(field_start));
        }

        else if (next_space - field_start != 0){
            line.push_back(input.substr(field_start, next_space - field_start));
        }

        field_start = next_space + 1;

    } while (next_space != std::string::npos);

    return line;

}



