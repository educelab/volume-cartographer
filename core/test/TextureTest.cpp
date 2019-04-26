#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <opencv2/imgcodecs.hpp>

#include "vc/core/types/Texture.hpp"
#include "vc/core/util/DateTime.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/testing/TestingUtils.hpp"

namespace vctest = volcart::testing;

/*
 * This fixtures builds objects for each of the test cases below that reference
 * the fixture as their second argument
 */
class EmptyConstructorTextureFixture : public ::testing::Test
{
public:
    EmptyConstructorTextureFixture()
    {
        // store metadata --> not needed
        _ID = _Texture.metadata().get<std::string>("id");
        _Type = _Texture.metadata().get<std::string>("type");
        _NumberOfImages = _Texture.numberOfImages();

        // strip off time information
        _TruncatedID = _ID[0 - 7];
    }

    volcart::Texture _Texture;
    std::string _ID, _Type, _TruncatedID;
    size_t _NumberOfImages;
};

/*
 * Used to test adding an image
 */
class TextureWithImageFixture : public ::testing::Test
{
public:
    TextureWithImageFixture()
    {
        // create cv::mat in loop and add to the texture
        for (int i = 2; i < 11; i++) {
            // add 9 identity matrix images to the texture
            _img = cv::Mat::eye(i, i, CV_16U);
            _Texture.addImage(_img);
        }
    }

    volcart::Texture _Texture;
    cv::Mat _img;
};

/*
 * Used to test retrieving intensity values
 * TextureWithUVMapToCheckIntensityFixture
 */
class TextureWithUVMapToCheckIntensityFixture : public ::testing::Test
{
public:
    TextureWithUVMapToCheckIntensityFixture()
    {
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

    volcart::UVMap _uvMap;
    volcart::Texture _Texture;
    cv::Mat _Gradient;
};

TEST_F(EmptyConstructorTextureFixture, TestTextureEmptyConstructor)
{

    // confirm texture has no images
    EXPECT_EQ(_NumberOfImages, 0);

    // check type is texture
    EXPECT_EQ(_Type, "texture");

    std::string TestDate, TruncatedTestDate;
    TestDate = volcart::DateTime()[0 - 7];
    TruncatedTestDate = TestDate[0 - 7];

    // check id
    EXPECT_EQ(TruncatedTestDate, _TruncatedID);
}

TEST_F(TextureWithImageFixture, AddImageToTextureTest)
{

    // check number of images
    EXPECT_EQ(_Texture.numberOfImages(), 9);

    // make sure no uvMap is in the texture
    EXPECT_EQ(_Texture.hasMap(), false);

    // check that each of the images added in fixture match identity mats
    for (size_t img_id = 0; img_id < _Texture.numberOfImages(); img_id++) {

        EXPECT_EQ(_Texture.image(img_id).rows, img_id + 2);
        EXPECT_EQ(_Texture.image(img_id).cols, img_id + 2);

        cv::Mat TestImage = cv::Mat::eye(img_id + 2, img_id + 2, CV_16U);

        // check data props of two matrices
        EXPECT_TRUE(
            vctest::CvMatEqual<uint16_t>(_Texture.image(img_id), TestImage));

        // assign img data
        uint16_t* matData = _Texture.image(img_id).ptr<uint16_t>();

        // step size --> helper
        size_t step = _Texture.image(img_id).step / sizeof(uint16_t);

        uint16_t value;
        for (int row = 0; row < _Texture.image(img_id).rows; row++) {
            for (int col = 0; col < _Texture.image(img_id).cols; col++) {

                value = matData[row * step + col];

                // check that the values match identity matrix
                if (row == col) {
                    EXPECT_EQ(value, 1);
                } else {
                    EXPECT_EQ(value, 0);
                }
            }
        }
    }
}

TEST_F(
    TextureWithUVMapToCheckIntensityFixture,
    IntensityRetrievalFromTextureImageTest)
{

    EXPECT_EQ(_Texture.numberOfImages(), 1);
    EXPECT_EQ(_Texture.image(0).empty(), false);

    // check dimensions
    EXPECT_EQ(_Texture.image(0).cols, 100);
    EXPECT_EQ(_Texture.image(0).rows, 100);

    // check intensity values
    EXPECT_EQ(_Texture.intensity(0), 65535);
    EXPECT_EQ(_Texture.intensity(1), 32767);
    EXPECT_EQ(_Texture.intensity(2), 32767);
    EXPECT_EQ(_Texture.intensity(3), 0);
}