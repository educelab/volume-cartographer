#include <gtest/gtest.h>

#include "vc/core/types/Transforms.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/types/VolumePkgVersion.hpp"
#include "vc/core/util/Logging.hpp"

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

    // Simple transform
    Transform3D::Pointer tfm1 = IdentityTransform::New();
    tfm1->source("a");
    tfm1->target("d");
    auto id1 = vpkg->addTransform(tfm1);

    // Simple transform (inverse)
    Transform3D::Pointer tfmI = IdentityTransform::New();
    tfmI->source("d");
    tfmI->target("a");
    auto idI = vpkg->addTransform(tfmI) + "*";

    // Long path: (a->b) (b->c) (c->d)
    Transform3D::Pointer tfm2 = IdentityTransform::New();
    tfm2->source("a");
    tfm2->target("b");
    auto id2 = vpkg->addTransform(tfm2);

    Transform3D::Pointer tfm3 = IdentityTransform::New();
    tfm3->source("b");
    tfm3->target("c");
    auto id3 = vpkg->addTransform(tfm3);

    Transform3D::Pointer tfm4 = IdentityTransform::New();
    tfm4->source("c");
    tfm4->target("d");
    auto id4 = vpkg->addTransform(tfm4);

    // Skipped path
    Transform3D::Pointer tfm5 = IdentityTransform::New();
    tfm5->source("z");
    tfm5->target("d");
    vpkg->addTransform(tfm5);

    // Inverted path: (f->a)' (e->f)' (d->e)'
    Transform3D::Pointer tfm6 = IdentityTransform::New();
    tfm6->source("f");
    tfm6->target("a");
    auto id6 = vpkg->addTransform(tfm6) + "*";

    Transform3D::Pointer tfm7 = IdentityTransform::New();
    tfm7->source("e");
    tfm7->target("f");
    auto id7 = vpkg->addTransform(tfm7) + "*";

    Transform3D::Pointer tfm8 = IdentityTransform::New();
    tfm8->source("d");
    tfm8->target("e");
    auto id8 = vpkg->addTransform(tfm8) + "*";

    // Note: Tests are non-overlapping paths
    auto tfms = vpkg->transform("a", "d");
    EXPECT_EQ(tfms.size(), 4);
    EXPECT_EQ(tfms[0].first, id1);
    EXPECT_EQ(tfms[1].first, idI);
    EXPECT_EQ(tfms[2].first, id2 + "->" + id3 + "->" + id4);
    EXPECT_EQ(tfms[3].first, id6 + "->" + id7 + "->" + id8);
}