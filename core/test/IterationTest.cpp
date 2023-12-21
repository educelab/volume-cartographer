#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "vc/core/util/Iteration.hpp"

using namespace volcart;

TEST(Iteration, RangeEndOnly)
{
    // Construct range and get iterator
    auto r = range(5);
    auto it = r.begin();

    // Do standard loop
    for (size_t i = 0; i < 5; i++) {

        // Compare to loop indices
        EXPECT_EQ(*it, i);

        // Increment iterator
        ++it;
    }
}

TEST(Iteration, RangeStartEnd)
{
    // Construct range and get iterator
    auto r = range(1, 5);
    auto it = r.begin();

    // Do standard loop
    for (size_t i = 1; i < 5; i++) {

        // Compare to loop indices
        EXPECT_EQ(*it, i);

        // Increment iterator
        ++it;
    }
}

TEST(Iteration, RangeStartEndStep)
{
    // Construct range and get iterator
    auto r = range(1, 5, 2);
    auto it = r.begin();

    // Do standard loop
    for (size_t i = 1; i < 5; i += 2) {

        // Compare to loop indices
        EXPECT_EQ(*it, i);

        // Increment iterator
        ++it;
    }
}

TEST(Iteration, RangeIteratorEquality)
{

    // Construct range and get iterator
    auto r = range(0, 10, 3);
    auto it = r.begin();

    // Make sure nothing prior to the end matches the end
    for (; it != r.end(); ++it) {
        EXPECT_NE(it, r.end());
    }

    // Make sure we match the end
    EXPECT_EQ(it, r.end());

    // Make sure incrementing keeps us at the end
    ++it;
    EXPECT_EQ(it, r.end());
}

TEST(Iteration, Range2DEndOnly)
{
    // Construct range and get iterator
    auto r = range2D(5, 5);
    auto it = r.begin();

    // Do standard 2D loop
    for (size_t y = 0; y < 5; y++) {
        for (size_t x = 0; x < 5; x++) {

            // Get iterator value
            auto pair = *it;
            const auto& v = pair.first;
            const auto& u = pair.second;

            // Compare to loop indices
            EXPECT_EQ(v, y);
            EXPECT_EQ(u, x);

            // Increment iterator
            ++it;
        }
    }
}

TEST(Iteration, Range2DStartEnd)
{
    // Construct range and get iterator
    auto r = range2D(1, 5, 1, 5);
    auto it = r.begin();

    // Do standard 2D loop
    for (size_t y = 1; y < 5; y++) {
        for (size_t x = 1; x < 5; x++) {

            // Get iterator value
            auto pair = *it;
            const auto& v = pair.first;
            const auto& u = pair.second;

            // Compare to loop indices
            EXPECT_EQ(v, y);
            EXPECT_EQ(u, x);

            // Increment iterator
            ++it;
        }
    }
}

TEST(Iteration, Range2DStartEndStep)
{
    // Construct range and get iterator
    auto r = range2D(1, 6, 1, 6, 2);
    auto it = r.begin();

    // Do standard 2D loop
    for (size_t y = 1; y < 6; y += 2) {
        for (size_t x = 1; x < 6; x += 2) {

            // Get iterator value
            auto pair = *it;
            const auto& v = pair.first;
            const auto& u = pair.second;

            // Compare to loop indices
            EXPECT_EQ(v, y);
            EXPECT_EQ(u, x);

            // Increment iterator
            ++it;
        }
    }
}

TEST(Iteration, Range2DFloat)
{
    float yStart{1.F};
    float yMax{5.F};
    float xStart{2.F};
    float xMax{6.F};
    float step{.5F};

    // Construct range and get iterator
    auto r = range2D(yStart, yMax, xStart, xMax, step);
    auto it = r.begin();

    for (float y = yStart; y < yMax; y += step) {
        for (float x = xStart; x < xMax; x += step) {
            // Get iterator value
            auto pair = *it;
            const auto& v = pair.first;
            const auto& u = pair.second;

            // Compare to loop indices
            EXPECT_FLOAT_EQ(v, y);
            EXPECT_FLOAT_EQ(u, x);

            // Increment iterator
            ++it;
        }
    }
}

TEST(Iteration, Range2DIteratorEquality)
{

    // Construct range and get iterator
    auto r = range2D(0, 10, 1, 11, 3);
    auto it = r.begin();

    // Make sure nothing prior to the end matches the end
    for (; it != r.end(); ++it) {
        EXPECT_NE(it, r.end());
    }

    // Make sure we match the end
    EXPECT_EQ(it, r.end());

    // Make sure incrementing keeps us at the end
    ++it;
    EXPECT_EQ(it, r.end());
}

TEST(Iteration, EnumerateLVal)
{
    std::vector<int> values{0, 1, 2, 3, 4};
    size_t idx{0};
    for (auto [index, value] : enumerate(values)) {
        EXPECT_EQ(index, idx);
        EXPECT_EQ(value, values[idx]);

        // Warning: Though we're iterating "by value", the value is always
        // a reference to the stored data.
        // Increment iterable's value and check that the container updated
        value++;
        EXPECT_EQ(value, values[idx]);

        ++idx;
    }

    // Make sure all elements were edited
    EXPECT_THAT(values, ::testing::ElementsAre(1, 2, 3, 4, 5));
}

TEST(Iteration, EnumerateRVal)
{
    std::vector<int> truth{0, 1, 2, 3, 4};
    size_t idx{0};
    for (const auto [index, value] :
         enumerate(std::vector<int>{0, 1, 2, 3, 4})) {
        EXPECT_EQ(index, idx);
        EXPECT_EQ(value, truth[idx]);
        ++idx;
    }
}

TEST(Iteration, EnumerateParamPack)
{
    std::vector<int> truth{0, 1, 2, 3, 4};
    size_t idx{0};
    for (const auto [index, value] : enumerate(0, 1, 2, 3, 4)) {
        EXPECT_EQ(index, idx);
        EXPECT_EQ(value, truth[idx]);
        ++idx;
    }
}

TEST(Iteration, EnumerateRange)
{
    std::vector<int> truth{0, 1, 2, 3, 4};
    size_t idx{0};
    for (auto pair : enumerate(range(5))) {
        const auto index = pair.first;
        const auto& value = pair.second;
        EXPECT_EQ(index, idx);
        EXPECT_EQ(value, truth[idx]);
        ++idx;
    }
}
