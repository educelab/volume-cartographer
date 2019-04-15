//
// Created by Seth Parker on 8/3/16.
//

#define BOOST_TEST_MODULE Metadata

#include "vc/core/types/Metadata.hpp"

#include <boost/test/unit_test.hpp>

#include "vc/core/util/Logging.hpp"

/************************************************************************************
 *                                                                                  *
 *  MetadataTest.cpp - tests the functionality of
 * /v-c/core/datatypes/Metadata.*  *
 *  The ultimate goal of this file is the following: *
 *                                                                                  *
 *  Check that we are able to create a Metadata, retrieve/set key-value pairs *
 *  appropriately, and serialize to and read from disk *
 *                                                                                  *
 *  This file is broken up into testing fixtures which initialize the *
 *  objects used in each of the three test cases. *
 *                                                                                  *
 *  1. Set/GetMetadata *
 *  2. IOMetadata *
 *                                                                                  *
 * Input: *
 *     No required inputs for this sample test. *
 *                                                                                  *
 * Test-Specific Output: *
 *     Specific test output only given on failure of any tests. Otherwise,
 * general  *
 *     number of testing errors is output. *
 *                                                                                  *
 * **********************************************************************************/

/***************
 *
 * FIXTURES
 *
 ***************/

struct EmptyMetadataFixture {

    EmptyMetadataFixture()
    {
        volcart::logger->debug("Creating empty Metadata fixture...");
    }

    ~EmptyMetadataFixture()
    {
        volcart::logger->debug("Destroying empty Metadata fixture...");
    }

    volcart::Metadata _metadata;
    int i = 1;
    double d = 1.0;
    std::string s = "value";
};

struct FilledMetadataFixture {

    FilledMetadataFixture()
    {
        volcart::logger->debug("Creating filled Metadata fixture...");
        _metadata.set("int", i);
        _metadata.set("double", d);
        _metadata.set("string", s);
    }

    ~FilledMetadataFixture()
    {
        volcart::logger->debug("Destroying filled Metadata fixture...");
    }

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

// Set/Get metadata
BOOST_FIXTURE_TEST_CASE(SetGetMetadata, EmptyMetadataFixture)
{
    // int
    _metadata.set("int", i);
    BOOST_CHECK_EQUAL(_metadata.get<int>("int"), i);

    // double
    _metadata.set("double", d);
    BOOST_CHECK_EQUAL(_metadata.get<double>("double"), d);

    // string
    _metadata.set("string", s);
    BOOST_CHECK_EQUAL(_metadata.get<std::string>("string"), s);
}

// Write/Read metadata
BOOST_FIXTURE_TEST_CASE(IOMetadata, FilledMetadataFixture)
{

    // Save the Metadata to file
    _metadata.setPath("FilledMetadata.json");
    _metadata.save();

    // Read the Metadata from file
    volcart::Metadata readMetadata("FilledMetadata.json");

    // Check that what we read matches what we expect
    BOOST_CHECK_EQUAL(readMetadata.get<int>("int"), _metadata.get<int>("int"));
    BOOST_CHECK_EQUAL(
        readMetadata.get<double>("double"), _metadata.get<double>("double"));
    BOOST_CHECK_EQUAL(
        readMetadata.get<std::string>("string"),
        _metadata.get<std::string>("string"));
}
