#include <gtest/gtest.h>

#include <bitset>

#include "vc/core/util/Flags.hpp"

using namespace volcart;

using Flag = std::bitset<4>;
static constexpr Flag NONE = 0;
static constexpr Flag OPTA = 1;
static constexpr Flag OPTB = 2;
static constexpr Flag OPTC = 4;

TEST(Flags, IsSet)
{
    Flag flag{NONE};
    EXPECT_FALSE(flag::is_set(flag));
    EXPECT_FALSE(flag::is_set(flag, OPTA));
    EXPECT_FALSE(flag::is_set(flag, OPTB));
    EXPECT_FALSE(flag::is_set(flag, OPTC));

    flag = OPTA;
    EXPECT_TRUE(flag::is_set(flag));
    EXPECT_TRUE(flag::is_set(flag, OPTA));
    EXPECT_FALSE(flag::is_set(flag, OPTA, OPTB, OPTC));

    flag = OPTA | OPTB;
    EXPECT_FALSE(flag::is_set(flag, OPTA, OPTB, OPTC));

    flag = OPTA | OPTB | OPTC;
    EXPECT_TRUE(flag::is_set(flag, OPTA, OPTB, OPTC));
}

TEST(Flags, SetSingle)
{
    Flag flag{NONE};
    flag::set(flag, OPTA);
    EXPECT_TRUE(flag::is_set(flag));
    EXPECT_TRUE(flag::is_set(flag, OPTA));
    EXPECT_FALSE(flag::is_set(flag, OPTB));
    EXPECT_FALSE(flag::is_set(flag, OPTC));

    flag::set(flag, OPTB);
    EXPECT_TRUE(flag::is_set(flag));
    EXPECT_TRUE(flag::is_set(flag, OPTA));
    EXPECT_TRUE(flag::is_set(flag, OPTB));
    EXPECT_FALSE(flag::is_set(flag, OPTC));

    flag::set(flag, OPTC);
    EXPECT_TRUE(flag::is_set(flag));
    EXPECT_TRUE(flag::is_set(flag, OPTA));
    EXPECT_TRUE(flag::is_set(flag, OPTB));
    EXPECT_TRUE(flag::is_set(flag, OPTC));
}

TEST(Flags, SetMultiple)
{
    Flag flag{NONE};
    flag::set(flag, OPTA, OPTB, OPTC);
    EXPECT_TRUE(flag::is_set(flag));
    EXPECT_TRUE(flag::is_set(flag, OPTA));
    EXPECT_TRUE(flag::is_set(flag, OPTB));
    EXPECT_TRUE(flag::is_set(flag, OPTC));
}

TEST(Flags, UnsetSingle)
{
    Flag flag{OPTA | OPTB | OPTC};
    flag::unset(flag, OPTA);
    EXPECT_TRUE(flag::is_set(flag));
    EXPECT_FALSE(flag::is_set(flag, OPTA));
    EXPECT_TRUE(flag::is_set(flag, OPTB));
    EXPECT_TRUE(flag::is_set(flag, OPTC));

    flag::unset(flag, OPTB);
    EXPECT_TRUE(flag::is_set(flag));
    EXPECT_FALSE(flag::is_set(flag, OPTA));
    EXPECT_FALSE(flag::is_set(flag, OPTB));
    EXPECT_TRUE(flag::is_set(flag, OPTC));

    flag::unset(flag, OPTC);
    EXPECT_FALSE(flag::is_set(flag));
    EXPECT_FALSE(flag::is_set(flag, OPTA));
    EXPECT_FALSE(flag::is_set(flag, OPTB));
    EXPECT_FALSE(flag::is_set(flag, OPTC));
}

TEST(Flags, UnsetMultiple)
{
    Flag flag{OPTA | OPTB | OPTC};
    flag::unset(flag, OPTA, OPTB, OPTC);
    EXPECT_FALSE(flag::is_set(flag));
    EXPECT_FALSE(flag::is_set(flag, OPTA));
    EXPECT_FALSE(flag::is_set(flag, OPTB));
    EXPECT_FALSE(flag::is_set(flag, OPTC));
}

TEST(Flags, UnsetAll)
{
    Flag flag{OPTA | OPTB | OPTC};
    flag::unset(flag);
    EXPECT_FALSE(flag::is_set(flag));
    EXPECT_FALSE(flag::is_set(flag, OPTA));
    EXPECT_FALSE(flag::is_set(flag, OPTB));
    EXPECT_FALSE(flag::is_set(flag, OPTC));
}

TEST(Flags, FlipSingle)
{
    Flag flag;
    flag::flip(flag, OPTA);
    EXPECT_TRUE(flag::is_set(flag));
    EXPECT_TRUE(flag::is_set(flag, OPTA));

    flag::flip(flag, OPTA);
    EXPECT_FALSE(flag::is_set(flag));
    EXPECT_FALSE(flag::is_set(flag, OPTA));
}

TEST(Flags, FlipMultiple)
{
    Flag flag{OPTB};
    flag::flip(flag, OPTA, OPTB, OPTC);
    EXPECT_TRUE(flag::is_set(flag, OPTA));
    EXPECT_FALSE(flag::is_set(flag, OPTB));
    EXPECT_TRUE(flag::is_set(flag, OPTC));

    flag::flip(flag, OPTA, OPTB, OPTC);
    EXPECT_FALSE(flag::is_set(flag, OPTA));
    EXPECT_TRUE(flag::is_set(flag, OPTB));
    EXPECT_FALSE(flag::is_set(flag, OPTC));
}