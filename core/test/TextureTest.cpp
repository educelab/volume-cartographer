//
// Created by Ryan Taber on 12/9/15.
//

#define BOOST_TEST_MODULE Texture

#include <boost/test/unit_test.hpp>
#include "core/types/Texture.h"

/************************************************************************************
 *                                                                                  *
 *  TextureTest.cpp - tests the functionality of
 * /v-c/core/datatypes/Texture.cpp  *
 *  The ultimate goal of this file is the following: *
 *                                                                                  *
 *    Check that we can create a texture, add an image to it, add a uvMap, and *
 *    retrieve intensity values successfully from a texture image *
 *                                                                                  *
 *                                                                                  *
 *  This file is broken up into a testing fixture(s) which initializes the *
 *  objects used in each of the three test cases. *
 *                                                                                  *
 *  1. TestTextureEmptyConstructor *
 *  2. AddImageToTextureTest *
 *  3. IntensityRetrievalFromTextureImageTest *
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

/*
 * This fixtures builds objects for each of the test cases below that reference
 * the fixture as their second argument
 */

struct EmptyConstructorTextureFixture {

    EmptyConstructorTextureFixture()
    {

        std::cerr << "Creating texture object using empty constructor..."
                  << std::endl;

        // store metadata --> not needed
        _ID = _Texture.metadata().get<std::string>("id");
        _Type = _Texture.metadata().get<std::string>("type");
        _NumberOfImages = _Texture.numberOfImages();

        // strip off time information
        _TruncatedID = _ID[0 - 7];
    }

    ~EmptyConstructorTextureFixture()
    {
        std::cerr << "Destroying texture object..." << std::endl;
    }

    volcart::Texture _Texture;
    std::string _ID, _Type, _TruncatedID;
    size_t _NumberOfImages;
};

/*
 * Used to test adding an image
 */
struct TextureWithImageFixture {

    TextureWithImageFixture()
    {

        std::cerr << "Creating texture with image(s)..." << std::endl;

        // create cv::mat in loop and add to the texture
        for (int i = 2; i < 11; i++) {
            // add 9 identity matrix images to the texture
            _img = cv::Mat::eye(i, i, CV_16U);
            _Texture.addImage(_img);
        }
    }

    ~TextureWithImageFixture()
    {
        std::cerr << "Destroying texture object..." << std::endl;
    }

    volcart::Texture _Texture;
    cv::Mat _img;
};

/*
 * Used to test retrieving intensity values
 */
struct TextureWithUVMapToCheckIntensityFixture {

    TextureWithUVMapToCheckIntensityFixture()
    {

        std::cerr << "Creating texture with uv map..." << std::endl;

        // read in and add image to the texture
        _Gradient = cv::imread("GenericGradient.tif", -1);
        _Texture.addImage(_Gradient);

        // 4-pt uv map
        _uvMap.set(0, cv::Vec2d(0, 0));
        _uvMap.set(1, cv::Vec2d(1, 0));
        _uvMap.set(2, cv::Vec2d(0, 1));
        _uvMap.set(3, cv::Vec2d(1, 1));

        // add uv map to texture
        _Texture.ppm().setUVMap(_uvMap);
    }

    ~TextureWithUVMapToCheckIntensityFixture()
    {
        std::cerr << "Destroying texture..." << std::endl;
    }

    volcart::UVMap _uvMap;
    volcart::Texture _Texture;
    cv::Mat _Gradient;
};

BOOST_FIXTURE_TEST_CASE(
    TestTextureEmptyConstructor, EmptyConstructorTextureFixture)
{

    // confirm texture has no images
    BOOST_CHECK_EQUAL(_NumberOfImages, 0);

    // check type is texture
    BOOST_CHECK_EQUAL(_Type, "texture");

    std::string TestDate, TruncatedTestDate;
    TestDate = volcart::DATE_TIME()[0 - 7];
    TruncatedTestDate = TestDate[0 - 7];

    // check id
    BOOST_CHECK_EQUAL(TruncatedTestDate, _TruncatedID);
}

BOOST_FIXTURE_TEST_CASE(AddImageToTextureTest, TextureWithImageFixture)
{

    // check number of images
    BOOST_CHECK_EQUAL(_Texture.numberOfImages(), 9);

    // make sure no uvMap is in the texture
    BOOST_CHECK_EQUAL(_Texture.hasMap(), false);

    // check that each of the images added in fixture match identity mats
    for (size_t img_id = 0; img_id < _Texture.numberOfImages(); img_id++) {

        BOOST_CHECK_EQUAL(_Texture.image(img_id).rows, img_id + 2);
        BOOST_CHECK_EQUAL(_Texture.image(img_id).cols, img_id + 2);

        cv::Mat TestImage = cv::Mat::eye(img_id + 2, img_id + 2, CV_16U);

        // check data props of two matrices
        BOOST_CHECK_EQUAL_COLLECTIONS(
            _Texture.image(img_id).datastart, _Texture.image(img_id).dataend,
            TestImage.datastart, TestImage.dataend);

        // assign img data
        ushort* matData = (ushort*)_Texture.image(img_id).data;

        // step size --> helper
        size_t step = _Texture.image(img_id).step / sizeof(ushort);

        ushort value;
        for (int row = 0; row < _Texture.image(img_id).rows; row++) {
            for (int col = 0; col < _Texture.image(img_id).cols; col++) {

                value = matData[row * step + col];

                // check that the values match identity matrix
                if (row == col) {
                    BOOST_CHECK_EQUAL(value, 1);
                } else {
                    BOOST_CHECK_EQUAL(value, 0);
                }
            }
        }
    }
}

BOOST_FIXTURE_TEST_CASE(
    IntensityRetrievalFromTextureImageTest,
    TextureWithUVMapToCheckIntensityFixture)
{

    BOOST_CHECK_EQUAL(_Texture.numberOfImages(), 1);
    BOOST_CHECK_EQUAL(_Texture.image(0).empty(), false);

    // check dimensions
    BOOST_CHECK_EQUAL(_Texture.image(0).cols, 100);
    BOOST_CHECK_EQUAL(_Texture.image(0).rows, 100);

    // check intensity values
    BOOST_CHECK_EQUAL(_Texture.intensity(0), 65535);
    BOOST_CHECK_EQUAL(_Texture.intensity(1), 32767);
    BOOST_CHECK_EQUAL(_Texture.intensity(2), 32767);
    BOOST_CHECK_EQUAL(_Texture.intensity(3), 0);
}
