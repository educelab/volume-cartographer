#include <gtest/gtest.h>

#include "vc/core/types/Transforms.hpp"
#include "vc/core/util/Iteration.hpp"

using namespace volcart;
namespace fs = volcart::filesystem;

TEST(Transforms, AffineSerialization)
{
    auto tfm = AffineTransform::New();
    tfm->source("abcdefgh");
    tfm->target("ijklmnop");

    const fs::path path{"vc_core_Transforms_AffineTransform.json"};
    AffineTransform::Save(path, tfm);
    auto result = AffineTransform::Load(path);
}

TEST(Transforms, AffineInvert)
{
    auto tfm = AffineTransform::New();

    tfm->rotate(90, 0, 0, 1);
    tfm = std::static_pointer_cast<AffineTransform>(tfm->invert());
}