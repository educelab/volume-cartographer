//
// Created by Ryan Taber on 3/3/16.
//

#ifndef VC_PREBUILT_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE simpleUV

#include <boost/test/unit_test.hpp>
#include "vc_defines.h"
#include "vc_datatypes.h"
#include "shapes.h"
#include "simpleUV.h"
#include "orderedPCDMesher.h"
#include <vtkSmartPointer.h>


/***************************************************************************************
 *                                                                                     *
 *  simpleUVTest.cpp - tests the functionality of /v-c/texturing/simpleUV.cpp          *
 *  The ultimate goal of this file is the following:                                   *
 *                                                                                     *
 *    Confirm that a uvMap created from mesh encodes the pointIDs of the               *
 *    mesh in <u,v> coords.                                                            *
 *                                                                                     *
 *  This file is broken up into a test fixture which initializes the objects           *
 *  used in each of the test cases.                                                    *
 *                                                                                     *
 *   1. PlaneSimpleUVTest                                                              *
 *   2. ArchSimpleUVTest                                                               *
 *                                                                                     *
 * Input:                                                                              *
 *     No required inputs for this sample test. All objects needed for the test cases  *
 *     are constructed and destroyed by the fixtures. Note, we're only testing the     *
 *     non-enclosed shapes here. Sphere, Cone and Cube have been omitted.              *
 *                                                                                     *
 * Test-Specific Output:                                                               *
 *     Specific test output only given on failure of any tests. Otherwise, general     *
 *     number of testing errors is output.                                             *
 *                                                                                     *
 * *************************************************************************************/


/*
 *
 *    FIXTURES
 *
 */

struct CreatePlaneSimpleUVFixture{

    CreatePlaneSimpleUVFixture(){

        std::cerr << "Creating Plane simple uv map..." << std::endl;

        //create ordered point cloud from shape and assign width and height
        //params: false noise and true ordered
        _in_PlanePointCloud =_Plane.pointCloudXYZ(false);
        _width = _in_PlanePointCloud.width;
        _height = _in_PlanePointCloud.height;

        _in_PlaneITKMesh = _Plane.itkMesh();

        //create uvMap from mesh, width and height
        _out_PlaneUVMap = volcart::texturing::simpleUV(_in_PlaneITKMesh, _width, _height);

    }

    ~CreatePlaneSimpleUVFixture(){std::cerr << "Destroying Plane simple uv map..." << std::endl;}

    //declare Plane mesh and width and height
    volcart::shapes::Plane _Plane;
    VC_MeshType::Pointer _in_PlaneITKMesh;
    pcl::PointCloud <pcl::PointXYZ> _in_PlanePointCloud;
    int _width, _height;

    //declare uvMap to hold output from simpleUV call
    volcart::UVMap _out_PlaneUVMap; 
};

struct CreateArchSimpleUVFixture{

    CreateArchSimpleUVFixture(){

        std::cerr << "Creating Arch simple uv map..." << std::endl;

        //create ordered point cloud from shape and assign width and height
        //params: false noise and true ordered
        _in_ArchPointCloud =_Arch.pointCloudXYZ(false);
        _width = _in_ArchPointCloud.width;
        _height = _in_ArchPointCloud.height;
        
        _in_ArchITKMesh = _Arch.itkMesh();

        //create uvMap from mesh, width and height
        _out_ArchUVMap = volcart::texturing::simpleUV(_in_ArchITKMesh, _width, _height);
    }

    ~CreateArchSimpleUVFixture(){std::cerr << "Destroying Arch simple uv map..." << std::endl;}

    //declare Arch mesh
    volcart::shapes::Arch _Arch;
    VC_MeshType::Pointer _in_ArchITKMesh;
    volcart::UVMap _out_ArchUVMap;

    pcl::PointCloud <pcl::PointXYZ> _in_ArchPointCloud;
    int _width, _height; 
    
};

/*
 *
 *    TEST CASES
 *
 */

BOOST_FIXTURE_TEST_CASE(PlaneSimpleUVTest, CreatePlaneSimpleUVFixture){

    //check size of uvMap and number of points in mesh
    BOOST_CHECK_EQUAL(_out_PlaneUVMap.size(), _in_PlaneITKMesh->GetNumberOfPoints());

    VC_PointsInMeshIterator pnt_id = _in_PlaneITKMesh->GetPoints()->Begin();
    double u,v;

    //check uvmap against original mesh input pointIDs
    for (auto it = 0; it < _out_PlaneUVMap.size(); it++){

        //get u,v for point in map
        u = _out_PlaneUVMap.get(it)[0];
        v = _out_PlaneUVMap.get(it)[1];

        //check to see the values of u,v correspond to mesh pointID
        BOOST_CHECK_EQUAL(u * _width, pnt_id.Index() % _width);
        BOOST_CHECK_EQUAL((v * _height * _width + (pnt_id.Index() % _width)), pnt_id.Index());

        pnt_id++;
    }

}

BOOST_FIXTURE_TEST_CASE(ArchSimpleUVTest, CreateArchSimpleUVFixture){

    //check size of uvMap and number of points in mesh
    BOOST_CHECK_EQUAL(_out_ArchUVMap.size(), _in_ArchITKMesh->GetNumberOfPoints());

    VC_PointsInMeshIterator pnt_id = _in_ArchITKMesh->GetPoints()->Begin();
    double u,v;
    
    //check uvmap vec against original mesh input
    for (auto it = 0; it < _out_ArchUVMap.size(); it++){

        //get u,v for point in map
        u = _out_ArchUVMap.get(it)[0];
        v = _out_ArchUVMap.get(it)[1];

        //check to see the values of u,v correspond to mesh pointID
        BOOST_CHECK_EQUAL(u * _width, pnt_id.Index() % _width);
        BOOST_CHECK_EQUAL((v * _height * _width + (pnt_id.Index() % _width)), pnt_id.Index());

        pnt_id++;

    }
}
    