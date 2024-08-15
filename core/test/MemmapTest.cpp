#include <gtest/gtest.h>

#include <fstream>
#include <string_view>

#include "vc/core/filesystem.hpp"
#include "vc/core/util/Memmap.hpp"

using namespace volcart;
namespace fs = filesystem;

TEST(Memmap, MapAndUnmap)
{
    // Create a file
    const fs::path file("vc_core_Memmap_MapAndUnmap.txt");
    std::ofstream out(file, std::ios::out);
    if (not out.is_open()) {
        throw std::runtime_error("Failed to open output file");
    }
    out << "FOO";
    out.close();

    // Memmap the file
    mmap_info info;
    EXPECT_NO_THROW(info = MemmapFile(file));
    EXPECT_TRUE(info);

    // Check the contents
    std::string_view str(static_cast<char*>(info.addr), info.size);
    EXPECT_EQ(str, "FOO");

    // Close the memmap
    EXPECT_EQ(UnmapFile(info), 0);
    EXPECT_FALSE(info);
}

TEST(Memmap, AutoUnmap)
{
    // Create a file
    const fs::path file("vc_core_Memmap_AutoUnmap.txt");
    std::ofstream out(file, std::ios::out);
    if (not out.is_open()) {
        throw std::runtime_error("Failed to open output file");
    }
    out << "BAR";
    out.close();

    // Memmap the file
    std::string_view str;
    char testChar = '0';
    {
        auto_mmap_info infoOuter;
        {
            auto_mmap_info infoInner;
            EXPECT_NO_THROW(infoInner = MemmapFile(file));
            EXPECT_TRUE(infoInner);

            // Check the contents
            str = std::string_view(static_cast<char*>(infoInner.addr), infoInner.size);
            EXPECT_EQ(str, "BAR");
            testChar = str[0];
            EXPECT_EQ(testChar, 'B');

            // Test move
            infoOuter = std::move(infoInner);
            EXPECT_FALSE(infoInner);
        }
        EXPECT_TRUE(infoOuter);
        testChar = str[1];
        EXPECT_EQ(testChar, 'A');
    }

    // Accessing the file should now fail and the assignment shouldn't happen
    EXPECT_EXIT(testChar = str[2], ::testing::KilledBySignal(SIGSEGV), "");
    EXPECT_EQ(testChar, 'A');
}