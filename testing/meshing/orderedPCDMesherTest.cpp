//
// Created by Ryan Taber on 11/6/15.
//

#ifndef VC_BOOST_STATIC_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE orderedPCDMesher

#include <boost/test/unit_test.hpp>
#include "vc_defines.h"
#include "shapes.h"
#include "orderedPCDMesher.h"
#include "parsingHelpers.h"


/************************************************************************************
 *                                                                                  *
 *  orderedPCDMesherTest.cpp - tests the functionality of                           *
 *  v-c/meshing/orderedPCDMesher.cpp with the ultimate goal of the following:       *
 *                                                                                  *
 *     Given the same input point cloud, does a saved PLY file match a current      *
 *     execution of orderedPCDMesher().                                             *
 *                                                                                  *
 *  This file is broken up into a test fixtures for each shape that initialize      *
 *  the objects used in subsequent fixture test cases.                              *
 *                                                                                  *
 *  Test Cases:                                                                     *
 *  1. CompareFixtureCreatedAndSavedPlaneOrderedPCDMesherData                       *
 *  2. CompareFixtureCreatedAndSavedCubeOrderedPCDMesherData                        *
 *  3. CompareFixtureCreatedAndSavedArchOrderedPCDMesherData                        *
 *  4. CompareFixtureCreatedAndSavedSphereOrderedPCDMesherData                      *
 *  5. CompareFixtureCreatedAndSavedConeOrderedPCDMesherData                        *
 *                                                                                  *                                                                            *
 *  Input:                                                                          *
 *     No required inputs for the test cases. Any test objects are created          *
 *     internally by orderedPCDFix() or within the test cases themselves.           *
 *                                                                                  *
 *  Test-Specific Output:                                                           *
 *     Specific test output only given on failure of any tests. Otherwise, general  *
 *     number of testing errors is output to console.                               *
 *                                                                                  *
 *  Miscellaneous:                                                                  *
 *     See the /testing/meshing wiki for more information on this test              *
 * **********************************************************************************/


/*
 * Plane fixture is commented, but comments removed from subsequent 
 * fixtures because they're using the same idea
 */

struct PlaneOrderedPCDFixture {

    PlaneOrderedPCDFixture() {

        //Create point cloud from mesh
        _in_PlanePointCloud = _Plane.pointCloudXYZRGB();

        //init point cloud ptr
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _in_PlaneCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        //convert pCloud to Ptr for orderedPCD() call
        *_in_PlaneCloudPtr = _in_PlanePointCloud;

        //assign outfile name
        _PlaneOutfile = "PlaneFixtureOrderedPCDMesher.ply";

        //call orderedPCD()
        volcart::meshing::orderedPCDMesher(_in_PlaneCloudPtr, _PlaneOutfile);

        //parse the newly created .ply file and save data in vectors
        volcart::testing::ParsingHelpers::parsePlyFile("PlaneFixtureOrderedPCDMesher.ply",
                                                                _FixtureCreatedPlanePoints, _FixtureCreatedPlaneCells);
        
        //parse out the saved Plane test file created by orderedPCDMesherExample.cpp
        volcart::testing::ParsingHelpers::parsePlyFile("PlaneOrderedPCDMesher.ply", _SavedPlanePoints, _SavedPlaneCells);
        
        std::cerr << "\nsetting up Plane OrderedPCDMesherTest objects" << std::endl;
    }

    ~PlaneOrderedPCDFixture(){ std::cerr << "\ncleaning up Plane OrderedPCDMesherTest objects" << std::endl; }

    pcl::PointCloud<pcl::PointXYZRGB> _in_PlanePointCloud;
    volcart::shapes::Plane _Plane;
    std::string _PlaneOutfile;

    std::vector<VC_Vertex> _SavedPlanePoints, _FixtureCreatedPlanePoints;
    std::vector<VC_Cell> _SavedPlaneCells, _FixtureCreatedPlaneCells;

};

struct CubeOrderedPCDFixture {

    CubeOrderedPCDFixture() {
        
        _in_CubePointCloud = _Cube.pointCloudXYZRGB();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _in_CubeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
        *_in_CubeCloudPtr = _in_CubePointCloud;
        _CubeOutfile = "CubeFixtureOrderedPCDMesher.ply";
        volcart::meshing::orderedPCDMesher(_in_CubeCloudPtr, _CubeOutfile);
        
        volcart::testing::ParsingHelpers::parsePlyFile("CubeFixtureOrderedPCDMesher.ply",
                                                       _FixtureCreatedCubePoints, _FixtureCreatedCubeCells);
        
        volcart::testing::ParsingHelpers::parsePlyFile("CubeOrderedPCDMesher.ply", _SavedCubePoints, _SavedCubeCells);

        std::cerr << "\nsetting up Cube OrderedPCDMesherTest objects" << std::endl;
    }

    ~CubeOrderedPCDFixture(){ std::cerr << "\ncleaning up Cube OrderedPCDMesherTest objects" << std::endl; }

    pcl::PointCloud<pcl::PointXYZRGB> _in_CubePointCloud;
    volcart::shapes::Cube _Cube;
    std::string _CubeOutfile;
    std::vector<VC_Vertex> _SavedCubePoints, _FixtureCreatedCubePoints;
    std::vector<VC_Cell> _SavedCubeCells, _FixtureCreatedCubeCells;

};

struct ArchOrderedPCDFixture {

    ArchOrderedPCDFixture() {

        _in_ArchPointCloud = _Arch.pointCloudXYZRGB();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _in_ArchCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
        *_in_ArchCloudPtr = _in_ArchPointCloud;
        _ArchOutfile = "ArchFixtureOrderedPCDMesher.ply";
        volcart::meshing::orderedPCDMesher(_in_ArchCloudPtr, _ArchOutfile);

        volcart::testing::ParsingHelpers::parsePlyFile("ArchFixtureOrderedPCDMesher.ply",
                                                       _FixtureCreatedArchPoints, _FixtureCreatedArchCells);

        volcart::testing::ParsingHelpers::parsePlyFile("ArchOrderedPCDMesher.ply", _SavedArchPoints, _SavedArchCells);

        std::cerr << "\nsetting up Arch OrderedPCDMesherTest objects" << std::endl;
    }

    ~ArchOrderedPCDFixture(){ std::cerr << "\ncleaning up Arch OrderedPCDMesherTest objects" << std::endl; }

    pcl::PointCloud<pcl::PointXYZRGB> _in_ArchPointCloud;
    volcart::shapes::Arch _Arch;
    std::string _ArchOutfile;
    std::vector<VC_Vertex> _SavedArchPoints, _FixtureCreatedArchPoints;
    std::vector<VC_Cell> _SavedArchCells, _FixtureCreatedArchCells;

};

struct SphereOrderedPCDFixture {

    SphereOrderedPCDFixture() {

        _in_SpherePointCloud = _Sphere.pointCloudXYZRGB();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _in_SphereCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
        *_in_SphereCloudPtr = _in_SpherePointCloud;
        _SphereOutfile = "SphereFixtureOrderedPCDMesher.ply";
        volcart::meshing::orderedPCDMesher(_in_SphereCloudPtr, _SphereOutfile);

        volcart::testing::ParsingHelpers::parsePlyFile("SphereFixtureOrderedPCDMesher.ply",
                                                       _FixtureCreatedSpherePoints, _FixtureCreatedSphereCells);

        volcart::testing::ParsingHelpers::parsePlyFile("SphereOrderedPCDMesher.ply", _SavedSpherePoints, _SavedSphereCells);

        std::cerr << "\nsetting up Sphere OrderedPCDMesherTest objects" << std::endl;
    }

    ~SphereOrderedPCDFixture(){ std::cerr << "\ncleaning up Sphere OrderedPCDMesherTest objects" << std::endl; }

    pcl::PointCloud<pcl::PointXYZRGB> _in_SpherePointCloud;
    volcart::shapes::Sphere _Sphere;
    std::string _SphereOutfile;
    std::vector<VC_Vertex> _SavedSpherePoints, _FixtureCreatedSpherePoints;
    std::vector<VC_Cell> _SavedSphereCells, _FixtureCreatedSphereCells;

};

struct ConeOrderedPCDFixture {

    ConeOrderedPCDFixture() {

        _in_ConePointCloud = _Cone.pointCloudXYZRGB();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _in_ConeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
        *_in_ConeCloudPtr = _in_ConePointCloud;
        _ConeOutfile = "ConeFixtureOrderedPCDMesher.ply";
        volcart::meshing::orderedPCDMesher(_in_ConeCloudPtr, _ConeOutfile);

        volcart::testing::ParsingHelpers::parsePlyFile("ConeFixtureOrderedPCDMesher.ply",
                                                       _FixtureCreatedConePoints, _FixtureCreatedConeCells);

        //saved testing file
        volcart::testing::ParsingHelpers::parsePlyFile("ConeOrderedPCDMesher.ply", _SavedConePoints, _SavedConeCells);

        std::cerr << "\nsetting up Cone OrderedPCDMesherTest objects" << std::endl;
    }

    ~ConeOrderedPCDFixture(){ std::cerr << "\ncleaning up Cone OrderedPCDMesherTest objects" << std::endl; }

    pcl::PointCloud<pcl::PointXYZRGB> _in_ConePointCloud;
    volcart::shapes::Cone _Cone;
    std::string _ConeOutfile;
    std::vector<VC_Vertex> _SavedConePoints, _FixtureCreatedConePoints;
    std::vector<VC_Cell> _SavedConeCells, _FixtureCreatedConeCells;

};

/*
 * Plane test case is commented but subsequent test cases are not
 */

BOOST_FIXTURE_TEST_CASE(CompareFixtureCreatedAndSavedPlaneOrderedPCDMesherData, PlaneOrderedPCDFixture){

    //Check number of points and cells from both files 
    BOOST_CHECK_EQUAL(_SavedPlanePoints.size(), _FixtureCreatedPlanePoints.size());
    BOOST_CHECK_EQUAL(_SavedPlaneCells.size(), _FixtureCreatedPlaneCells.size());

    //Loop through the points within the clouds to check equivalency
    for (int p = 0; p < _SavedPlanePoints.size(); p++){

        //check x,y,z,nx,ny,nz,s,t,r,g,b values
        BOOST_CHECK_EQUAL(_SavedPlanePoints[p].x, _FixtureCreatedPlanePoints[p].x);
        BOOST_CHECK_EQUAL(_SavedPlanePoints[p].y, _FixtureCreatedPlanePoints[p].y);
        BOOST_CHECK_EQUAL(_SavedPlanePoints[p].z, _FixtureCreatedPlanePoints[p].z);
        BOOST_CHECK_EQUAL(_SavedPlanePoints[p].nx, _FixtureCreatedPlanePoints[p].nx);
        BOOST_CHECK_EQUAL(_SavedPlanePoints[p].ny, _FixtureCreatedPlanePoints[p].ny);
        BOOST_CHECK_EQUAL(_SavedPlanePoints[p].nz, _FixtureCreatedPlanePoints[p].nz);
        BOOST_CHECK_EQUAL(_SavedPlanePoints[p].s, _FixtureCreatedPlanePoints[p].s);
        BOOST_CHECK_EQUAL(_SavedPlanePoints[p].t, _FixtureCreatedPlanePoints[p].t);
        BOOST_CHECK_EQUAL(_SavedPlanePoints[p].r, _FixtureCreatedPlanePoints[p].r);
        BOOST_CHECK_EQUAL(_SavedPlanePoints[p].g, _FixtureCreatedPlanePoints[p].g);
        BOOST_CHECK_EQUAL(_SavedPlanePoints[p].b, _FixtureCreatedPlanePoints[p].b);

    }

    for (int f = 0; f < _SavedPlaneCells.size(); f++){

        //check the vertices within each face
        BOOST_CHECK_EQUAL(_SavedPlaneCells[f].v1, _FixtureCreatedPlaneCells[f].v1);
        BOOST_CHECK_EQUAL(_SavedPlaneCells[f].v2, _FixtureCreatedPlaneCells[f].v2);
        BOOST_CHECK_EQUAL(_SavedPlaneCells[f].v3, _FixtureCreatedPlaneCells[f].v3);
    }
}

BOOST_FIXTURE_TEST_CASE(CompareFixtureCreatedAndSavedCubeOrderedPCDMesherData, CubeOrderedPCDFixture){

    BOOST_CHECK_EQUAL(_SavedCubePoints.size(), _FixtureCreatedCubePoints.size());
    BOOST_CHECK_EQUAL(_SavedCubeCells.size(), _FixtureCreatedCubeCells.size());

    for (int p = 0; p < _SavedCubePoints.size(); p++){

        BOOST_CHECK_EQUAL(_SavedCubePoints[p].x, _FixtureCreatedCubePoints[p].x);
        BOOST_CHECK_EQUAL(_SavedCubePoints[p].y, _FixtureCreatedCubePoints[p].y);
        BOOST_CHECK_EQUAL(_SavedCubePoints[p].z, _FixtureCreatedCubePoints[p].z);
        BOOST_CHECK_EQUAL(_SavedCubePoints[p].nx, _FixtureCreatedCubePoints[p].nx);
        BOOST_CHECK_EQUAL(_SavedCubePoints[p].ny, _FixtureCreatedCubePoints[p].ny);
        BOOST_CHECK_EQUAL(_SavedCubePoints[p].nz, _FixtureCreatedCubePoints[p].nz);
        BOOST_CHECK_EQUAL(_SavedCubePoints[p].s, _FixtureCreatedCubePoints[p].s);
        BOOST_CHECK_EQUAL(_SavedCubePoints[p].t, _FixtureCreatedCubePoints[p].t);
        BOOST_CHECK_EQUAL(_SavedCubePoints[p].r, _FixtureCreatedCubePoints[p].r);
        BOOST_CHECK_EQUAL(_SavedCubePoints[p].g, _FixtureCreatedCubePoints[p].g);
        BOOST_CHECK_EQUAL(_SavedCubePoints[p].b, _FixtureCreatedCubePoints[p].b);

    }

    for (int f = 0; f < _SavedCubeCells.size(); f++){

        BOOST_CHECK_EQUAL(_SavedCubeCells[f].v1, _FixtureCreatedCubeCells[f].v1);
        BOOST_CHECK_EQUAL(_SavedCubeCells[f].v2, _FixtureCreatedCubeCells[f].v2);
        BOOST_CHECK_EQUAL(_SavedCubeCells[f].v3, _FixtureCreatedCubeCells[f].v3);
    }
}

BOOST_FIXTURE_TEST_CASE(CompareFixtureCreatedAndSavedArchOrderedPCDMesherData, ArchOrderedPCDFixture){

    BOOST_CHECK_EQUAL(_SavedArchPoints.size(), _FixtureCreatedArchPoints.size());
    BOOST_CHECK_EQUAL(_SavedArchCells.size(), _FixtureCreatedArchCells.size());

    for (int p = 0; p < _SavedArchPoints.size(); p++){

        BOOST_CHECK_EQUAL(_SavedArchPoints[p].x, _FixtureCreatedArchPoints[p].x);
        BOOST_CHECK_EQUAL(_SavedArchPoints[p].y, _FixtureCreatedArchPoints[p].y);
        BOOST_CHECK_EQUAL(_SavedArchPoints[p].z, _FixtureCreatedArchPoints[p].z);
        BOOST_CHECK_EQUAL(_SavedArchPoints[p].nx, _FixtureCreatedArchPoints[p].nx);
        BOOST_CHECK_EQUAL(_SavedArchPoints[p].ny, _FixtureCreatedArchPoints[p].ny);
        BOOST_CHECK_EQUAL(_SavedArchPoints[p].nz, _FixtureCreatedArchPoints[p].nz);
        BOOST_CHECK_EQUAL(_SavedArchPoints[p].s, _FixtureCreatedArchPoints[p].s);
        BOOST_CHECK_EQUAL(_SavedArchPoints[p].t, _FixtureCreatedArchPoints[p].t);
        BOOST_CHECK_EQUAL(_SavedArchPoints[p].r, _FixtureCreatedArchPoints[p].r);
        BOOST_CHECK_EQUAL(_SavedArchPoints[p].g, _FixtureCreatedArchPoints[p].g);
        BOOST_CHECK_EQUAL(_SavedArchPoints[p].b, _FixtureCreatedArchPoints[p].b);

    }

    for (int f = 0; f < _SavedArchCells.size(); f++){

        BOOST_CHECK_EQUAL(_SavedArchCells[f].v1, _FixtureCreatedArchCells[f].v1);
        BOOST_CHECK_EQUAL(_SavedArchCells[f].v2, _FixtureCreatedArchCells[f].v2);
        BOOST_CHECK_EQUAL(_SavedArchCells[f].v3, _FixtureCreatedArchCells[f].v3);
    }
}

BOOST_FIXTURE_TEST_CASE(CompareFixtureCreatedAndSavedSphereOrderedPCDMesherData, SphereOrderedPCDFixture){

    BOOST_CHECK_EQUAL(_SavedSpherePoints.size(), _FixtureCreatedSpherePoints.size());
    BOOST_CHECK_EQUAL(_SavedSphereCells.size(), _FixtureCreatedSphereCells.size());

    for (int p = 0; p < _SavedSpherePoints.size(); p++){

        BOOST_CHECK_EQUAL(_SavedSpherePoints[p].x, _FixtureCreatedSpherePoints[p].x);
        BOOST_CHECK_EQUAL(_SavedSpherePoints[p].y, _FixtureCreatedSpherePoints[p].y);
        BOOST_CHECK_EQUAL(_SavedSpherePoints[p].z, _FixtureCreatedSpherePoints[p].z);
        BOOST_CHECK_EQUAL(_SavedSpherePoints[p].nx, _FixtureCreatedSpherePoints[p].nx);
        BOOST_CHECK_EQUAL(_SavedSpherePoints[p].ny, _FixtureCreatedSpherePoints[p].ny);
        BOOST_CHECK_EQUAL(_SavedSpherePoints[p].nz, _FixtureCreatedSpherePoints[p].nz);
        BOOST_CHECK_EQUAL(_SavedSpherePoints[p].s, _FixtureCreatedSpherePoints[p].s);
        BOOST_CHECK_EQUAL(_SavedSpherePoints[p].t, _FixtureCreatedSpherePoints[p].t);
        BOOST_CHECK_EQUAL(_SavedSpherePoints[p].r, _FixtureCreatedSpherePoints[p].r);
        BOOST_CHECK_EQUAL(_SavedSpherePoints[p].g, _FixtureCreatedSpherePoints[p].g);
        BOOST_CHECK_EQUAL(_SavedSpherePoints[p].b, _FixtureCreatedSpherePoints[p].b);

    }

    for (int f = 0; f < _SavedSphereCells.size(); f++){

        BOOST_CHECK_EQUAL(_SavedSphereCells[f].v1, _FixtureCreatedSphereCells[f].v1);
        BOOST_CHECK_EQUAL(_SavedSphereCells[f].v2, _FixtureCreatedSphereCells[f].v2);
        BOOST_CHECK_EQUAL(_SavedSphereCells[f].v3, _FixtureCreatedSphereCells[f].v3);
    }
}


BOOST_FIXTURE_TEST_CASE(CompareFixtureCreatedAndSavedConeOrderedPCDMesherData, ConeOrderedPCDFixture){

    BOOST_CHECK_EQUAL(_SavedConePoints.size(), _FixtureCreatedConePoints.size());
    BOOST_CHECK_EQUAL(_SavedConeCells.size(), _FixtureCreatedConeCells.size());

    for (int p = 0; p < _SavedConePoints.size(); p++){

        BOOST_CHECK_EQUAL(_SavedConePoints[p].x, _FixtureCreatedConePoints[p].x);
        BOOST_CHECK_EQUAL(_SavedConePoints[p].y, _FixtureCreatedConePoints[p].y);
        BOOST_CHECK_EQUAL(_SavedConePoints[p].z, _FixtureCreatedConePoints[p].z);
        BOOST_CHECK_EQUAL(_SavedConePoints[p].nx, _FixtureCreatedConePoints[p].nx);
        BOOST_CHECK_EQUAL(_SavedConePoints[p].ny, _FixtureCreatedConePoints[p].ny);
        BOOST_CHECK_EQUAL(_SavedConePoints[p].nz, _FixtureCreatedConePoints[p].nz);
        BOOST_CHECK_EQUAL(_SavedConePoints[p].s, _FixtureCreatedConePoints[p].s);
        BOOST_CHECK_EQUAL(_SavedConePoints[p].t, _FixtureCreatedConePoints[p].t);
        BOOST_CHECK_EQUAL(_SavedConePoints[p].r, _FixtureCreatedConePoints[p].r);
        BOOST_CHECK_EQUAL(_SavedConePoints[p].g, _FixtureCreatedConePoints[p].g);
        BOOST_CHECK_EQUAL(_SavedConePoints[p].b, _FixtureCreatedConePoints[p].b);

    }

    for (int f = 0; f < _SavedConeCells.size(); f++){

        BOOST_CHECK_EQUAL(_SavedConeCells[f].v1, _FixtureCreatedConeCells[f].v1);
        BOOST_CHECK_EQUAL(_SavedConeCells[f].v2, _FixtureCreatedConeCells[f].v2);
        BOOST_CHECK_EQUAL(_SavedConeCells[f].v3, _FixtureCreatedConeCells[f].v3);
    }
}