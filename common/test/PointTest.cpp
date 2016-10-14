#define BOOST_TEST_MODULE PointTest

#include <array>
#include <boost/test/unit_test.hpp>
#include "common/types/Point.h"
#include "testing/testingUtils.h"

using namespace volcart;
using namespace volcart::testing;

BOOST_AUTO_TEST_CASE(BasicConstructPointTest)
{
    // Basic construction using variadic templates
    Point3i p1(0, 1, 2);
    BOOST_CHECK_EQUAL(p1[0], 0);
    BOOST_CHECK_EQUAL(p1[1], 1);
    BOOST_CHECK_EQUAL(p1[2], 2);
}

// Copy construct
BOOST_AUTO_TEST_CASE(CopyConstructPointTest)
{
    Point3i p1{0, 1, 2};
    Point3i p3{p1};
    BOOST_CHECK_EQUAL(p3[0], p1[0]);
    BOOST_CHECK_EQUAL(p3[1], p1[1]);
    BOOST_CHECK_EQUAL(p3[2], p1[2]);
}

// Move construct
// Note: can't use values from p3 since it's left in an undefined state
BOOST_AUTO_TEST_CASE(MoveConstructPointTest)
{
    Point3i p3{0, 1, 2};
    Point3i p4{std::move(p3)};
    BOOST_CHECK_EQUAL(p4[0], 0);
    BOOST_CHECK_EQUAL(p4[1], 1);
    BOOST_CHECK_EQUAL(p4[2], 2);
}

// Construct from cv::Vec
BOOST_AUTO_TEST_CASE(CvVecConstructPointTest)
{
    cv::Vec3i v(1, 2, 3);
    Point3i p1(v);
    BOOST_CHECK_EQUAL(p1[0], v(0));
    BOOST_CHECK_EQUAL(p1[1], v(1));
    BOOST_CHECK_EQUAL(p1[2], v(2));
}

// Fill construct
BOOST_AUTO_TEST_CASE(FillConstructPointTest)
{
    auto p5 = Point3i::fill(3);
    BOOST_CHECK_EQUAL(p5[0], 3);
    BOOST_CHECK_EQUAL(p5[1], 3);
    BOOST_CHECK_EQUAL(p5[2], 3);
}

// operator= copy construction
BOOST_AUTO_TEST_CASE(OperatorEqualCopyConstructPointTest)
{
    Point3i p5{0, 1, 2};
    auto p6 = p5;
    BOOST_CHECK_EQUAL(p6[0], p5[0]);
    BOOST_CHECK_EQUAL(p6[1], p5[1]);
    BOOST_CHECK_EQUAL(p6[2], p5[2]);
}

// operator= move construction
BOOST_AUTO_TEST_CASE(OperatorEqualMoveConstructPointTest)
{
    Point3i p5{0, 1, 2};
    auto p6 = std::move(p5);
    BOOST_CHECK_EQUAL(p6[0], 0);
    BOOST_CHECK_EQUAL(p6[1], 1);
    BOOST_CHECK_EQUAL(p6[2], 2);
}

BOOST_AUTO_TEST_CASE(SelfPointAdditionTest)
{
    // += Point
    Point3i p1{0, 1, 2};
    p1 += Point3i{1, 2, 3};
    BOOST_CHECK_EQUAL(p1[0], 1);
    BOOST_CHECK_EQUAL(p1[1], 3);
    BOOST_CHECK_EQUAL(p1[2], 5);

    // += Scalar
    p1 += 5;
    BOOST_CHECK_EQUAL(p1[0], 6);
    BOOST_CHECK_EQUAL(p1[1], 8);
    BOOST_CHECK_EQUAL(p1[2], 10);
}

BOOST_AUTO_TEST_CASE(SelfPointSubtractionTest)
{
    // -= Point
    Point3i p1{7, 8, 9};
    p1 -= Point3i{1, 2, 3};
    BOOST_CHECK_EQUAL(p1[0], 6);
    BOOST_CHECK_EQUAL(p1[1], 6);
    BOOST_CHECK_EQUAL(p1[2], 6);

    // -= Scalar
    p1 -= 5;
    BOOST_CHECK_EQUAL(p1[0], 1);
    BOOST_CHECK_EQUAL(p1[1], 1);
    BOOST_CHECK_EQUAL(p1[2], 1);
}

BOOST_AUTO_TEST_CASE(SelfPointMultiplicationTest)
{
    // *= Scalar
    Point3i p1{0, 1, 2};
    p1 *= -5;
    BOOST_CHECK_EQUAL(p1[0], 0);
    BOOST_CHECK_EQUAL(p1[1], -5);
    BOOST_CHECK_EQUAL(p1[2], -10);
}

BOOST_AUTO_TEST_CASE(SelfPointDivisionTest)
{
    // /= Scalar
    Point3i p1{0, -5, -10};
    p1 /= 10;
    BOOST_CHECK_EQUAL(p1[0], 0);
    BOOST_CHECK_EQUAL(p1[1], 0);
    BOOST_CHECK_EQUAL(p1[2], -1);
}

BOOST_AUTO_TEST_CASE(PointAdditionTest)
{
    Point3i p1{1, 2, 3};
    Point3i p2{4, 5, 6};

    // Addition; point + point
    auto p3 = p1 + p2;
    BOOST_CHECK_EQUAL(p3[0], 5);
    BOOST_CHECK_EQUAL(p3[1], 7);
    BOOST_CHECK_EQUAL(p3[2], 9);
    p3 = p2 + p1;
    BOOST_CHECK_EQUAL(p3[0], 5);
    BOOST_CHECK_EQUAL(p3[1], 7);
    BOOST_CHECK_EQUAL(p3[2], 9);
}

// Subtraction: point - point
BOOST_AUTO_TEST_CASE(PointSubtractionTest)
{
    Point3i p1{1, 2, 3};
    Point3i p2{4, 5, 6};

    auto p4 = p1 - p2;
    BOOST_CHECK_EQUAL(p4[0], -3);
    BOOST_CHECK_EQUAL(p4[1], -3);
    BOOST_CHECK_EQUAL(p4[2], -3);
    p4 = p2 - p1;
    BOOST_CHECK_EQUAL(p4[0], 3);
    BOOST_CHECK_EQUAL(p4[1], 3);
    BOOST_CHECK_EQUAL(p4[2], 3);
}

// Addition: scalar
BOOST_AUTO_TEST_CASE(PointScalarAdditionTest)
{
    Point3i p1{1, 2, 3};

    auto p4 = p1 + 5;
    BOOST_CHECK_EQUAL(p4[0], 6);
    BOOST_CHECK_EQUAL(p4[1], 7);
    BOOST_CHECK_EQUAL(p4[2], 8);
    p4 = 5 + p1;
    BOOST_CHECK_EQUAL(p4[0], 6);
    BOOST_CHECK_EQUAL(p4[1], 7);
    BOOST_CHECK_EQUAL(p4[2], 8);
}

// Subtraction: scalar
BOOST_AUTO_TEST_CASE(PointScalarSubtractionTest)
{
    Point3i p1{1, 2, 3};

    auto p4 = p1 - 5;
    BOOST_CHECK_EQUAL(p4[0], -4);
    BOOST_CHECK_EQUAL(p4[1], -3);
    BOOST_CHECK_EQUAL(p4[2], -2);
    p4 = 5 - p1;
    BOOST_CHECK_EQUAL(p4[0], 4);
    BOOST_CHECK_EQUAL(p4[1], 3);
    BOOST_CHECK_EQUAL(p4[2], 2);
}

BOOST_AUTO_TEST_CASE(PointScalarMultiplicationTest)
{
    Point3i p1{1, 2, 3};

    auto p4 = p1 * 5;
    BOOST_CHECK_EQUAL(p4[0], 5);
    BOOST_CHECK_EQUAL(p4[1], 10);
    BOOST_CHECK_EQUAL(p4[2], 15);
    p4 = 5 * p1;
    BOOST_CHECK_EQUAL(p4[0], 5);
    BOOST_CHECK_EQUAL(p4[1], 10);
    BOOST_CHECK_EQUAL(p4[2], 15);
}

BOOST_AUTO_TEST_CASE(PointScalarDivisionTest)
{
    Point3i p1{5, 10, 15};

    auto p4 = p1 / 5;
    BOOST_CHECK_EQUAL(p4[0], 1);
    BOOST_CHECK_EQUAL(p4[1], 2);
    BOOST_CHECK_EQUAL(p4[2], 3);
}

BOOST_AUTO_TEST_CASE(PointCompareEqualtyTest)
{
    Point3i p1{1, 2, 3};
    Point3i p2{1, 2, 3};
    BOOST_CHECK_EQUAL(p1, p2);
    p2 += 1;
    BOOST_CHECK_NE(p1, p2);
}

BOOST_AUTO_TEST_CASE(PointNormTest)
{
    // Norm should be == sqrt(50) ~ 7.07106781187
    Point3i p1{3, 4, 5};
    double n = p1.norm();
    SmallOrClose(n, 7.07106781187);
}

BOOST_AUTO_TEST_CASE(PointIteratorsTest)
{
    Point3i p1{1, 2, 3};
    BOOST_CHECK_EQUAL(p1.front(), 1);
    BOOST_CHECK_EQUAL(p1.back(), 3);
}

BOOST_AUTO_TEST_CASE(ZeroPointAsBytesTest)
{
    Point3i p0{0, 0, 0};
    char* bytes = p0.bytes();
    size_t nbytes = sizeof(int) * p0.size();
    for (size_t i = 0; i < nbytes; ++i) {
        BOOST_CHECK_EQUAL(bytes[i], static_cast<char>(0));
    }
}

BOOST_AUTO_TEST_CASE(NonzeroPointAsBytesTest)
{
    Point3i p0{1, 2, 3};
    // Little-endian: LSB is lower in memory
    const std::array<char, 12> expected = {
        0x1, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0, 0x0,
    };
    const char* bytes = p0.bytes();
    size_t nbytes = sizeof(int) * p0.size();
    for (size_t i = 0; i < nbytes; ++i) {
        BOOST_CHECK_EQUAL(bytes[i], expected[i]);
    }
}
