#include <gtest/gtest.h>

#include "vc/core/types/Transforms.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/testing/TestingUtils.hpp"

using namespace volcart;
using namespace volcart::testing;
namespace fs = volcart::filesystem;

TEST(Transform, AffineClone)
{
    // Create transform
    auto tfm = AffineTransform::New();
    tfm->source("abcdefgh");
    tfm->target("ijklmnop");
    tfm->rotate(90, cv::Vec3d{0, 0, 1});
    tfm->translate(1, 2, 3);

    AffineTransform test = *tfm;

    auto result = std::static_pointer_cast<AffineTransform>(tfm->clone());
    // Compare equality
    EXPECT_EQ(result->type(), tfm->type());
    EXPECT_EQ(result->source(), tfm->source());
    EXPECT_EQ(result->target(), tfm->target());
    EXPECT_EQ(result->params(), tfm->params());
}

TEST(Transforms, AffineSerialization)
{
    // Create transform
    auto tfm = AffineTransform::New();
    tfm->source("abcdefgh");
    tfm->target("ijklmnop");
    tfm->rotate(90, cv::Vec3d{0, 0, 1});
    tfm->translate(1, 2, 3);

    // Write to disk
    const fs::path path{"vc_core_Transforms_AffineTransform.json"};
    AffineTransform::Save(path, tfm);

    // Read from disk
    auto result =
        std::static_pointer_cast<AffineTransform>(AffineTransform::Load(path));

    // Compare equality
    EXPECT_EQ(result->type(), tfm->type());
    EXPECT_EQ(result->source(), tfm->source());
    EXPECT_EQ(result->target(), tfm->target());
    EXPECT_EQ(result->params(), tfm->params());
}

TEST(Transforms, AffineGetSetParams)
{
    // Custom parameters
    using Parameters = AffineTransform::Parameters;
    Parameters params{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

    // Test that transform returns the same parameters we gave it
    auto tfm = AffineTransform::New();
    tfm->params(params);
    EXPECT_EQ(tfm->params(), params);
}

TEST(Transforms, AffineResetClear)
{
    // Build a transform
    auto tfm = AffineTransform::New();
    tfm->source("abc");
    tfm->target("def");
    tfm->translate(1, 2, 3);
    tfm->scale(2);
    tfm->rotate(90, cv::Vec3d{0, 0, 1});
    auto params = tfm->params();

    // Test that reset only affects parameters
    tfm->reset();
    EXPECT_EQ(tfm->source(), "abc");
    EXPECT_EQ(tfm->target(), "def");
    EXPECT_EQ(tfm->params(), AffineTransform::Parameters::eye());

    // Test that clear resets everything
    tfm->params(params);
    tfm->clear();
    EXPECT_EQ(tfm->source(), "");
    EXPECT_EQ(tfm->target(), "");
    EXPECT_EQ(tfm->params(), AffineTransform::Parameters::eye());
}

TEST(Transforms, AffineTranslate)
{
    // Simple transform
    auto tfm = AffineTransform::New();
    tfm->translate(1, 2, 3);

    // Test point
    const cv::Vec3d orig{0, 2, 0};

    // Apply as point
    auto result = tfm->applyPoint(orig);
    SmallOrClose(result, {1, 4, 3});

    // Apply as vector
    result = tfm->applyVector(orig);
    SmallOrClose(result, orig);

    // Apply as unit vector
    result = tfm->applyUnitVector(orig);
    SmallOrClose(result, orig / cv::norm(orig));
}

TEST(Transforms, AffineRotate)
{
    // Simple transform
    auto tfm = AffineTransform::New();
    tfm->rotate(90, 0, 0, 1);

    // Test point
    const cv::Vec3d orig{0, 10, 0};

    // Apply as point
    auto result = tfm->applyPoint(orig);
    cv::Vec3d expected{-10, 0, 0};
    SmallOrClose(result, expected);

    // Apply as vector
    result = tfm->applyVector(orig);
    SmallOrClose(result, expected);

    // Apply as unit vector
    result = tfm->applyUnitVector(orig);
    SmallOrClose(result, expected / 10.);
}

TEST(Transforms, AffineScale)
{
    // Simple transform
    auto tfm = AffineTransform::New();
    tfm->scale(8, 9, 10);

    // Test point
    const cv::Vec3d orig{1, 1, 1};

    // Apply as point
    auto result = tfm->applyPoint(orig);
    cv::Vec3d expected{8, 9, 10};
    SmallOrClose(result, expected);

    // Apply as vector
    result = tfm->applyVector(orig);
    SmallOrClose(result, expected);

    // Apply as unit vector
    result = tfm->applyUnitVector(orig);
    SmallOrClose(result, expected / cv::norm(expected));
}

TEST(Transforms, AffineCompound)
{
    auto tfm = AffineTransform::New();
    tfm->scale(5);
    tfm->rotate(90, 0, 0, 1);
    tfm->translate(0, 10, 0);

    auto result = tfm->applyPoint({0, 1, 0});
    SmallOrClose(result, {-5., 10., 0.});
}

TEST(Transforms, AffinePointAndNormal)
{
    // Simple transform
    auto tfm = AffineTransform::New();
    tfm->translate(1, 2, 3);

    // Test pt + unit normal
    cv::Vec6d ptN{0, 0, 0, 0, 1, 0};
    auto resultPN = tfm->applyPointAndNormal(ptN);
    EXPECT_EQ(resultPN, cv::Vec6d(1, 2, 3, 0, 1, 0));

    // Test pt + non-unit normal
    ptN[4] = 10.;
    resultPN = tfm->applyPointAndNormal(ptN);
    EXPECT_EQ(resultPN, cv::Vec6d(1, 2, 3, 0, 1, 0));

    // Test pt + non-unit normal (no normalization)
    resultPN = tfm->applyPointAndNormal(ptN, false);
    EXPECT_EQ(resultPN, cv::Vec6d(1, 2, 3, 0, 10, 0));
}

TEST(Transforms, AffineInvert)
{
    // Original point
    const cv::Vec3d orig{0, 1, 1};

    // Test invertible
    auto tfm = AffineTransform::New();
    EXPECT_TRUE(tfm->invertible());

    // Test translation forward
    tfm->translate(1, 2, 3);
    auto result = tfm->applyPoint(orig);
    SmallOrClose(result, {1, 3, 4});
    // Test translation inverse
    auto inv = tfm->invert();
    result = inv->applyPoint(result);
    SmallOrClose(result, orig);

    // Test rotation forward
    tfm->reset();
    tfm->rotate(90, 0, 0, 1);
    result = tfm->applyPoint(orig);
    SmallOrClose(result, {-1, 0, 1});
    // Test rotation inverse
    inv = tfm->invert();
    result = inv->applyPoint(result);
    SmallOrClose(result, orig);

    // Test scale forward
    tfm->reset();
    tfm->scale(1, 2, 3);
    result = tfm->applyPoint(orig);
    SmallOrClose(result, {0, 2, 3});
    // Test scale inverse
    inv = tfm->invert();
    result = inv->applyPoint(result);
    SmallOrClose(result, orig);

    // Test compound inverse
    tfm->reset();
    tfm->scale(1, 2, 3);
    tfm->rotate(90, 0, 0, 1);
    tfm->translate(1, 2, 3);
    result = tfm->applyPoint(orig);
    SmallOrClose(result, {-1, 2, 6});

    // Test compound inverse
    inv = tfm->invert();
    result = inv->applyPoint(result);
    SmallOrClose(result, orig);
}