#include <gtest/gtest.h>

#include "vc/core/types/Transforms.hpp"
#include "vc/core/util/Iteration.hpp"

using namespace volcart;
namespace fs = volcart::filesystem;

TEST(Transforms, AffineSerialization)
{
    // Create transform
    auto tfm = AffineTransform::New();
    tfm->source("abcdefgh");
    tfm->target("ijklmnop");
    tfm->rotate(90, cv::Vec3d{0, 0, 1}).translate(1, 2, 3);

    // Write to disk
    const fs::path path{"vc_core_Transforms_AffineTransform.json"};
    AffineTransform::Save(path, tfm);

    // Read from disk
    auto result =
        std::static_pointer_cast<AffineTransform>(AffineTransform::Load(path));

    // Compare equality
    EXPECT_EQ(result->source(), tfm->source());
    EXPECT_EQ(result->target(), tfm->target());
    EXPECT_EQ(result->params(), tfm->params());
}

TEST(Transforms, AffineTranslate)
{
    AffineTransform tfm;
    tfm.translate(1, 2, 3);

    // Test point
    const cv::Vec3d orig{0, 0, 0};

    auto result = tfm.applyPoint(orig);
    EXPECT_EQ(result, cv::Vec3d(1, 2, 3));

    result = tfm.applyVector(orig);
    EXPECT_EQ(result, orig);

    // Test pt + unit normal
    cv::Vec6d ptN{0, 0, 0, 0, 1, 0};
    auto resultPN = tfm.applyPointAndNormal(ptN);
    EXPECT_EQ(resultPN, cv::Vec6d(1, 2, 3, 0, 1, 0));

    // Test pt + non-unit normal
    ptN[4] = 10.;
    resultPN = tfm.applyPointAndNormal(ptN);
    EXPECT_EQ(resultPN, cv::Vec6d(1, 2, 3, 0, 1, 0));

    // Test pt + non-unit normal (no normalization)
    resultPN = tfm.applyPointAndNormal(ptN, false);
    EXPECT_EQ(resultPN, cv::Vec6d(1, 2, 3, 0, 10, 0));
}

TEST(Transforms, AffineInvert)
{
    auto tfm = AffineTransform::New();

    tfm->rotate(90, 0, 0, 1);
    std::cout << *tfm;
    std::cout << "\n";
    tfm = std::static_pointer_cast<AffineTransform>(tfm->invert());
    std::cout << *tfm;
}