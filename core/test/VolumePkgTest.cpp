#include <gtest/gtest.h>

#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/types/VolumePkgVersion.hpp"
#include "vc/core/types/Transforms.hpp"

using namespace volcart;
namespace fs = filesystem;

TEST(VolumePkg, TransformByID)
{
    // Initialize new volpkg
    fs::path p("TransformByID.volpkg");
    fs::remove_all(p);
    auto vpkg = VolumePkg::New(p, VOLPKG_VERSION_LATEST);

    // Create transform
    auto tfm = AffineTransform::New();
    tfm->source("0");
    tfm->target("1");

    // Add to volpkg and get ID
    auto id = vpkg->addTransform(tfm);

    // Get result
    auto res = vpkg->transform(id);

    // Pointer comparison (addTransform does not clone)
    EXPECT_EQ(tfm, res);
}

TEST(VolumePkg, TransformBySimplePath)
{
    // Initialize new volpkg
    fs::path p("TransformBySimplePath.volpkg");
    fs::remove_all(p);
    auto vpkg = VolumePkg::New(p, VOLPKG_VERSION_LATEST);

    Transform3D::Pointer tfm1 = AffineTransform::New();
    tfm1->source("a");
    tfm1->target("b");
    auto id1 = vpkg->addTransform(tfm1);

    Transform3D::Pointer tfm2 = IdentityTransform::New();
    tfm2->source("b");
    tfm2->target("a");
    auto id2 = vpkg->addTransform(tfm2);

    Transform3D::Pointer tfm3 = CompositeTransform::New();
    tfm3->source("b");
    tfm3->target("c");
    vpkg->addTransform(tfm3);

    auto tfms = vpkg->transform("a", "b");
    EXPECT_EQ(tfms.size(), 2);
    EXPECT_EQ(tfms[0].first, id1);
    EXPECT_EQ(tfms[1].first, id2 + "*");
}

TEST(VolumePkg, TransformByComplexPath)
{
    // Initialize new volpkg
    fs::path p("TransformByComplexPath.volpkg");
    fs::remove_all(p);
    auto vpkg = VolumePkg::New(p, VOLPKG_VERSION_LATEST);

    Transform3D::Pointer tfm1 = IdentityTransform::New();
    tfm1->source("a");
    tfm1->target("d");
    auto id1 = vpkg->addTransform(tfm1);

    Transform3D::Pointer tfm2 = IdentityTransform::New();
    tfm2->source("a");
    tfm2->target("b");
    auto id2 = vpkg->addTransform(tfm2);

    Transform3D::Pointer tfm3 = IdentityTransform::New();
    tfm3->source("b");
    tfm3->target("c");
    auto id3 = vpkg->addTransform(tfm3);

    Transform3D::Pointer tfm4 = IdentityTransform::New();
    tfm4->source("d");
    tfm4->target("c");
    auto id4 = vpkg->addTransform(tfm4);

    Transform3D::Pointer tfm5 = IdentityTransform::New();
    tfm5->source("e");
    tfm5->target("d");
    vpkg->addTransform(tfm5);

    auto tfms = vpkg->transform("a", "d");
    for(const auto& [id, tfm] : tfms) {
        std::cout << id << ": " << tfm->source() << "->" << tfm->target() << "\n";
    }
    EXPECT_EQ(tfms.size(), 2);
    EXPECT_EQ(tfms[0].first, id1);
    EXPECT_EQ(tfms[1].first, id2 + "->" + id3 + "->" + id4 + "*");
}