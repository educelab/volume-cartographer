//
// Created by Seth Parker on 11/19/15.
//
#pragma once

#include "core/shapes/ShapePrimitive.hpp"
#include "core/vc_defines.hpp"

namespace volcart
{
namespace shapes
{
class Plane : public ShapePrimitive
{
public:
    Plane(int width = 5, int height = 5)
    {

        // generate the points along the y-axis
        double y = 0;
        for (double x = 0; x < width; ++x) {
            for (double z = 0; z < height; ++z) {
                addVertex_(x, y, z);
            }
        }

        // generate the cells
        for (int i = 1; i < height; ++i) {
            for (int j = 1; j < width; ++j) {
                int v1, v2, v3, v4;
                v1 = i * width + j;
                v2 = v1 - 1;
                v3 = v2 - width;
                v4 = v1 - width;
                addCell_(v1, v2, v3);
                addCell_(v1, v3, v4);
            }
        }

        // Set this as an ordered mesh
        orderedPoints_ = true;
        orderedWidth_ = width;
        orderedHeight_ = height;

    }  // Constructor

};  // Plane
}  // shapes
}  // volcart
