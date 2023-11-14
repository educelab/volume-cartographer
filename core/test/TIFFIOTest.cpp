#include <gtest/gtest.h>

#include <iostream>
#include <limits>

#include <opencv2/core.hpp>

#include "vc/core/io/TIFFIO.hpp"

using namespace volcart::tiffio;
namespace fs = volcart::filesystem;

TEST(TIFFIO, WriteRead8UC1)
{
    using ElemT = std::uint8_t;
    using PixelT = ElemT;
    auto cvType = CV_8UC1;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, WriteRead8UC2)
{
    using ElemT = std::uint8_t;
    using PixelT = cv::Vec<ElemT, 2>;
    auto cvType = CV_8UC2;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, WriteRead8UC3)
{
    using ElemT = std::uint8_t;
    using PixelT = cv::Vec<ElemT, 3>;
    auto cvType = CV_8UC3;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, WriteRead8UC4)
{
    using ElemT = std::uint8_t;
    using PixelT = cv::Vec<ElemT, 4>;
    auto cvType = CV_8UC4;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, WriteRead8SC1)
{
    using ElemT = std::int8_t;
    using PixelT = ElemT;
    auto cvType = CV_8SC1;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, WriteRead8SC2)
{
    using ElemT = std::int8_t;
    using PixelT = cv::Vec<ElemT, 2>;
    auto cvType = CV_8SC2;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, Write8SC3)
{
    using ElemT = std::int8_t;
    auto cvType = CV_8SC3;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_Write_" + cv::typeToString(cvType) + ".tif");
    EXPECT_THROW(WriteTIFF(imgPath, img), std::runtime_error);
}

TEST(TIFFIO, Write8SC4)
{
    using ElemT = std::int8_t;
    auto cvType = CV_8SC4;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_Write_" + cv::typeToString(cvType) + ".tif");
    EXPECT_THROW(WriteTIFF(imgPath, img), std::runtime_error);
}

TEST(TIFFIO, WriteRead16UC1)
{
    using ElemT = std::uint16_t;
    using PixelT = ElemT;
    auto cvType = CV_16UC1;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, WriteRead16UC2)
{
    using ElemT = std::uint16_t;
    using PixelT = cv::Vec<ElemT, 2>;
    auto cvType = CV_16UC2;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, WriteRead16UC3)
{
    using ElemT = std::uint16_t;
    using PixelT = cv::Vec<ElemT, 3>;
    auto cvType = CV_16UC3;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, WriteRead16UC4)
{
    using ElemT = std::uint16_t;
    using PixelT = cv::Vec<ElemT, 4>;
    auto cvType = CV_16UC4;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, WriteRead16SC1)
{
    using ElemT = std::int16_t;
    using PixelT = ElemT;
    auto cvType = CV_16SC1;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, WriteRead16SC2)
{
    using ElemT = std::int16_t;
    using PixelT = cv::Vec<ElemT, 2>;
    auto cvType = CV_16SC2;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, Write16SC3)
{
    using ElemT = std::int16_t;
    auto cvType = CV_16SC3;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_Write_" + cv::typeToString(cvType) + ".tif");
    EXPECT_THROW(WriteTIFF(imgPath, img), std::runtime_error);
}

TEST(TIFFIO, Write16SC4)
{
    using ElemT = std::int16_t;
    auto cvType = CV_16SC4;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_Write_" + cv::typeToString(cvType) + ".tif");
    EXPECT_THROW(WriteTIFF(imgPath, img), std::runtime_error);
}

TEST(TIFFIO, WriteRead32SC1)
{
    using ElemT = std::int32_t;
    using PixelT = ElemT;
    auto cvType = CV_32SC1;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, WriteRead32SC2)
{
    using ElemT = std::int32_t;
    using PixelT = cv::Vec<ElemT, 2>;
    auto cvType = CV_32SC2;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, Write32SC3)
{
    using ElemT = std::int32_t;
    auto cvType = CV_32SC3;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_Write_" + cv::typeToString(cvType) + ".tif");
    EXPECT_THROW(WriteTIFF(imgPath, img), std::runtime_error);
}

TEST(TIFFIO, Write32SC4)
{
    using ElemT = std::int32_t;
    auto cvType = CV_32SC4;
    auto low = std::numeric_limits<ElemT>::min();
    auto high = std::numeric_limits<ElemT>::max();

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_Write_" + cv::typeToString(cvType) + ".tif");
    EXPECT_THROW(WriteTIFF(imgPath, img), std::runtime_error);
}

TEST(TIFFIO, WriteRead32FC1)
{
    using ElemT = std::float_t;
    using PixelT = ElemT;
    auto cvType = CV_32FC1;
    auto low = 0.F;
    auto high = 1.F;

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, WriteRead32FC2)
{
    using ElemT = std::float_t;
    using PixelT = cv::Vec<ElemT, 2>;
    auto cvType = CV_32FC2;
    auto low = 0.F;
    auto high = 1.F;

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, WriteRead32FC3)
{
    using ElemT = std::float_t;
    using PixelT = cv::Vec<ElemT, 3>;
    auto cvType = CV_32FC3;
    auto low = 0.F;
    auto high = 1.F;

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}

TEST(TIFFIO, WriteRead32FC4)
{
    using ElemT = std::float_t;
    using PixelT = cv::Vec<ElemT, 4>;
    auto cvType = CV_32FC4;
    auto low = 0.F;
    auto high = 1.F;

    cv::Mat img(10, 10, cvType);
    cv::randu(img, low, high);

    fs::path imgPath(
        "vc_core_TIFFIO_WriteRead_" + cv::typeToString(cvType) + ".tif");
    WriteTIFF(imgPath, img);
    auto result = ReadTIFF(imgPath);

    EXPECT_EQ(result.size, img.size);
    EXPECT_EQ(result.type(), img.type());

    auto equal = std::equal(
        result.begin<PixelT>(), result.end<PixelT>(), img.begin<PixelT>());
    EXPECT_TRUE(equal);
}