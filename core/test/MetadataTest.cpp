#include <gtest/gtest.h>

#include <string>

#include "vc/core/types/Metadata.hpp"
#include "vc/core/util/Logging.hpp"

/***************
 *
 * FIXTURES
 *
 ***************/

class Metadata_Empty : public ::testing::Test
{
public:
    volcart::Metadata meta;
    int i{1};
    double d{1.0};
    std::string s{"value"};
};

class Metadata_Filled : public Metadata_Empty
{
public:
    Metadata_Filled()
    {
        meta.set("int", i);
        meta.set("double", d);
        meta.set("string", s);
    }
};

/***************
 *
 * TEST CASES
 *
 ***************/

// Set/Get metadata
TEST_F(Metadata_Empty, SetGetMetadata)
{
    // int
    meta.set("int", i);
    EXPECT_EQ(meta.get<int>("int"), i);

    // double
    meta.set("double", d);
    EXPECT_EQ(meta.get<double>("double"), d);

    // string
    meta.set("string", s);
    EXPECT_EQ(meta.get<std::string>("string"), s);
}

// Write/Read metadata
TEST_F(Metadata_Filled, IOMetadata)
{
    // Save the Metadata to file
    meta.setPath("FilledMetadata.json");
    meta.save();

    // Read the Metadata from file
    volcart::Metadata read("FilledMetadata.json");

    // Check that what we read matches what we expect
    EXPECT_EQ(read.get<int>("int"), meta.get<int>("int"));
    EXPECT_EQ(read.get<double>("double"), meta.get<double>("double"));
    EXPECT_EQ(read.get<std::string>("string"), meta.get<std::string>("string"));
}