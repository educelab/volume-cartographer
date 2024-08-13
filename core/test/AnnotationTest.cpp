#include <gtest/gtest.h>

#include <educelab/core/utils/Flags.hpp>
#include <educelab/core/utils/Iteration.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/Annotation.hpp"

using namespace volcart;
namespace fs = filesystem;
namespace el = educelab;
namespace flag = el::flag;

TEST(Annotation, WriteReadSet)
{
    // Create annotation set
    AnnotationSet expected{10};
    std::vector<Annotation> row;
    row.reserve(10);
    AnnotationFlag f;
    for (const auto y : el::range(10)) {
        if (y % 5 == 0) {
            flag::set(f, ANNO_ANCHOR, ANNO_MANUAL);
        } else {
            flag::unset(f);
        }
        row.clear();

        for (const auto x : el::range(10)) {
            auto p = static_cast<double>(x);
            row.emplace_back(y, f, p, p);
        }
        expected.pushRow(row);
    }

    // Write the annotation set
    fs::path p{"vc_core_AnnotationTest_WriteReadSet.vcano"};
    EXPECT_NO_THROW(WriteAnnotationSet(p, expected));

    // Read the annotation set
    AnnotationSet result;
    EXPECT_NO_THROW(result = ReadAnnotationSet(p));

    // Compare the expected to the result
    EXPECT_EQ(result.width(), expected.width());
    EXPECT_EQ(result.height(), expected.height());
    for (const auto [y, x] : el::range2D(10, 10)) {
        auto& r = result(y, x);
        auto& e = expected(y, x);
        EXPECT_EQ(r.index, e.index);
        EXPECT_EQ(r.flags, e.flags);
        if (y % 5 == 0) {
            EXPECT_TRUE(flag::is_set(r.flags, ANNO_ANCHOR, ANNO_MANUAL));
        } else {
            EXPECT_FALSE(flag::is_set(r.flags));
        }
        EXPECT_EQ(r.pt, e.pt);
    }
}