//
// Created by Seth Parker on 8/3/16.
//

#ifndef VC_BOOST_STATIC_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE Metadata

#include <boost/test/unit_test.hpp>
#include "vc_datatypes.h"

/************************************************************************************
 *                                                                                  *
 *  MetadataTest.cpp - tests the functionality of /v-c/common/datatypes/Metadata.*  *
 *  The ultimate goal of this file is the following:                                *
 *                                                                                  *
 *  Check that we are able to create a Metadata, retrieve/set key-value pairs       *
 *  appropriately, and serialize to and read from disk                              *
 *                                                                                  *
 *  This file is broken up into testing fixtures which initialize the               *
 *  objects used in each of the three test cases.                                   *
 *                                                                                  *
 *  1. SetMetadata                                                                  *
 *  2. GetMetadata                                                                  *
 *  3. IOMetadata                                                                   *
 *                                                                                  *
 * Input:                                                                           *
 *     No required inputs for this sample test.                                     *
 *                                                                                  *
 * Test-Specific Output:                                                            *
 *     Specific test output only given on failure of any tests. Otherwise, general  *
 *     number of testing errors is output.                                          *
 *                                                                                  *
 * **********************************************************************************/


/***************
 *
 * FIXTURES
 *
 ***************/

struct EmptyMetadataFixture {

    EmptyMetadataFixture(){
        std::cerr << "Creating empty Metadata fixture..." << std::endl;
    }

    ~EmptyMetadataFixture(){ std::cerr << "Destroying empty Metadata fixture..." << std::endl;}

    volcart::Metadata _metadata;
};

struct FilledMetadataFixture {

    FilledMetadataFixture(){
        std::cerr << "Creating filled Metadata fixture..." << std::endl;
        _metadata.set("int", i);
        _metadata.set("double", d);
        _metadata.set("string", s);
    }

    ~FilledMetadataFixture(){ std::cerr << "Destroying filled Metadata fixture..." << std::endl;}

    volcart::Metadata _metadata;
    int i = 1;
    double d = 1.0;
    std::string s = "value";
};

/***************
 *
 * TEST CASES
 *
 ***************/

// Set metadata
BOOST_FIXTURE_TEST_CASE(SetMetadata, EmptyMetadataFixture) {
    // int
    _metadata.set("int", 1);

    // double
    _metadata.set("double", 1.0);

    // string
    _metadata.set("string", "value");
}

// Get metadata
BOOST_FIXTURE_TEST_CASE(GetMetadata, FilledMetadataFixture) {
    // int
    BOOST_CHECK_EQUAL( _metadata.get<int>("int"), i);

    // double
    BOOST_CHECK_EQUAL( _metadata.get<double>("double"), d);

    // string
    BOOST_CHECK_EQUAL( _metadata.get<std::string>("string"), s);
}

// Write/Read metadata
BOOST_FIXTURE_TEST_CASE(IOMetadata, FilledMetadataFixture) {

    // Save the Metadata to file
    _metadata.setPath("FilledMetadata.json");
    _metadata.save();

    // Read the Metadata from file
    volcart::Metadata readMetadata("FilledMetadata.json");

    // Check that what we read matches what we expect
    BOOST_CHECK_EQUAL( readMetadata.get<int>("int"), _metadata.get<int>("int") );
    BOOST_CHECK_EQUAL( readMetadata.get<double>("double"), _metadata.get<double>("double") );
    BOOST_CHECK_EQUAL( readMetadata.get<std::string>("string"), _metadata.get<std::string>("string") );
}