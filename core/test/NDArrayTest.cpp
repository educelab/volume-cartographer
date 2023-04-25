#include <iostream>

#include <gtest/gtest.h>

#include "vc/core/types/NDArray.hpp"

namespace vc = volcart;

//// Simplify with typedefs as needed ////
using IntArray = vc::NDArray<int>;
using Idx = IntArray::IndexType;

TEST(NDArray, Array4D)
{
    // Constructor
    IntArray::Extent size4{5, 4, 3, 2};
    IntArray array4(4, size4);

    // Assignment
    array4(4, 3, 2, 1) = 18;

    // Retrieval
    EXPECT_EQ(array4(4, 3, 2, 1), 18);
}

TEST(NDArray, Array3D)
{
    //// 3D array ////
    // Initialize the Extents with a list
    IntArray array3(3, 4, 3, 2);

    // Expand the array
    array3.setExtents(4, 4, 4);

    // Fill the array
    int val = 18;
    for (Idx z = 0; z < array3.extents()[0]; z++) {
        for (Idx y = 0; y < array3.extents()[1]; y++) {
            for (Idx x = 0; x < array3.extents()[2]; x++) {
                array3(z, y, x) = val++;
            }
        }
    }

    EXPECT_EQ(array3(3, 3, 3), 81);
}

TEST(NDArray, Array2D)
{

    // Fill a 3D array
    IntArray array3(3, 4, 4, 4);
    int val = 0;
    for (auto& i : array3) {
        i = val++;
    }

    // Get 2D array by slicing the 3D array
    IntArray array2 = array3.slice(3);
    EXPECT_EQ(array2(3, 3), 63);

    // Get 2D array by flattening the 3D array
    IntArray arrayFlattened = array3;
    IntArray::Flatten(arrayFlattened, 2);
    EXPECT_EQ(arrayFlattened(15, 3), 63);
}

TEST(NDArray, BadConstructor)
{
    // Get raw data
    std::vector<int> data = {0, 1, 2};

    // Try to make bad array
    EXPECT_THROW(
        IntArray array2_3(2, {5, 3}, data.begin(), data.end()),
        std::invalid_argument);
}