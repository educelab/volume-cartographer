//
// Created by Melissa Shankle on 12/16/15.
//
#include <cmath>

#include "core/shapes/Sphere.hpp"
#include "core/vc_defines.hpp"

using namespace volcart;
using namespace volcart::shapes;

Sphere::Sphere(float /*radius*/, int recursionLevel)
{
    // create 12 vertices of a icosahedron
    double t = (1.0 + std::sqrt(5.0)) / 2.0;

    // must make everything on the unit sphere, so divide by length
    double length = sqrt(1 + t * t);

    addVertex_(-1 / length, t / length, 0);
    addVertex_(1 / length, t / length, 0);
    addVertex_(-1 / length, -t / length, 0);
    addVertex_(1 / length, -t / length, 0);

    addVertex_(0, -1 / length, t / length);
    addVertex_(0, 1 / length, t / length);
    addVertex_(0, -1 / length, -t / length);
    addVertex_(0, 1 / length, -t / length);

    addVertex_(t / length, 0, -1 / length);
    addVertex_(t / length, 0, 1 / length);
    addVertex_(-t / length, 0, -1 / length);
    addVertex_(-t / length, 0, 1 / length);

    // create 20 triangles of the icosahedron
    std::vector<Cell> tmpCells;

    // 5 faces around point 0
    tmpCells.emplace_back(Cell(0, 11, 5));
    tmpCells.emplace_back(Cell(0, 5, 1));
    tmpCells.emplace_back(Cell(0, 1, 7));
    tmpCells.emplace_back(Cell(0, 7, 10));
    tmpCells.emplace_back(Cell(0, 10, 11));

    // 5 adjacent faces
    tmpCells.emplace_back(Cell(1, 5, 9));
    tmpCells.emplace_back(Cell(5, 11, 4));
    tmpCells.emplace_back(Cell(11, 10, 2));
    tmpCells.emplace_back(Cell(10, 7, 6));
    tmpCells.emplace_back(Cell(7, 1, 8));

    // 5 faces around point 3
    tmpCells.emplace_back(Cell(3, 9, 4));
    tmpCells.emplace_back(Cell(3, 4, 2));
    tmpCells.emplace_back(Cell(3, 2, 6));
    tmpCells.emplace_back(Cell(3, 6, 8));
    tmpCells.emplace_back(Cell(3, 8, 9));

    // 5 adjacent faces
    tmpCells.emplace_back(Cell(4, 9, 5));
    tmpCells.emplace_back(Cell(2, 4, 11));
    tmpCells.emplace_back(Cell(6, 2, 10));
    tmpCells.emplace_back(Cell(8, 6, 7));
    tmpCells.emplace_back(Cell(9, 8, 1));

    // refine triangles
    for (int i = 0; i < recursionLevel; i++) {
        std::vector<Cell> tmpCells2;

        for (auto cell : tmpCells) {
            // replace using 4 triangles
            int a = midpoint_(cell.v1, cell.v2);
            int b = midpoint_(cell.v2, cell.v3);
            int c = midpoint_(cell.v3, cell.v1);

            tmpCells2.emplace_back(Cell(cell.v1, a, c));
            tmpCells2.emplace_back(Cell(cell.v2, b, a));
            tmpCells2.emplace_back(Cell(cell.v3, c, b));
            tmpCells2.emplace_back(Cell(a, b, c));
        }
        tmpCells = tmpCells2;
    }

    // add triangles to mesh
    for (auto cell : tmpCells) {
        addCell_(cell.v1, cell.v2, cell.v3);
    }

    orderedPoints_ = false;
    orderedWidth_ = orderedHeight_ = 0;
}

int Sphere::midpoint_(int p1, int p2)
{
    // Generate unique id
    bool firstIsSmaller = p1 < p2;
    int smallerIndex = firstIsSmaller ? p1 : p2;
    int greaterIndex = firstIsSmaller ? p2 : p1;
    auto key =
        std::to_string(smallerIndex) + "-" + std::to_string(greaterIndex);

    // Find unique id and return if cached
    auto found = indexCache_.find(key);
    if (found != indexCache_.end()) {
        return found->second;
    }

    // calculate
    double midX, midY, midZ;

    midX = (points_[p1].x + points_[p2].x) / 2;
    midY = (points_[p1].y + points_[p2].y) / 2;
    midZ = (points_[p1].z + points_[p2].z) / 2;
    ;

    // ensure unit sphere by dividing each point by length
    double length = std::sqrt(midX * midX + midY * midY + midZ * midZ);
    double x = midX / length;
    double y = midY / length;
    double z = midZ / length;

    // Add to points list and cache
    addVertex_(x, y, z);
    indexCache_.insert({key, points_.size() - 1});

    return (points_.size() - 1);
}
