#pragma once

/**
 * @file SimpleMesh.hpp
 * @author Seth Parker
 * @date 7/7/15
 *
 * @brief Simple mesh structure
 *
 * @ingroup Types
 */

#include <cstdint>
#include <iostream>

namespace volcart
{
/** Generic mesh structure */
struct SimpleMesh {
    /** Generic vertex structure */
    struct Vertex {
        double x, y, z, nx, ny, nz, s, t;
        int r, g, b, faceCount;
    };

    /** Generic triangular face structure */
    struct Cell {
        std::uint64_t v1, v2, v3;
        Cell() = default;
        Cell(std::uint64_t p1, std::uint64_t p2, std::uint64_t p3)
            : v1{p1}, v2{p2}, v3{p3}
        {
        }
    };

    std::vector<Vertex> verts;
    std::vector<Cell> faces;
};

}  // namespace volcart
