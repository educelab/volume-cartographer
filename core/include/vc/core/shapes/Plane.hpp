#pragma once

/** @file */

#include "ShapePrimitive.hpp"

namespace volcart::shapes
{
/**
 * @author Seth Parker
 * @date 11/19/15
 *
 * @brief Planar surface shape
 *
 * This class generates a planar surface parallel to the XZ plane at y = 0.
 * Points are 1 unit apart.
 *
 * The shape will have a number of points equal to width x height. The points
 * generated by this class are ordered.
 *
 * @ingroup Shapes
 */
class Plane : public ShapePrimitive
{
public:
    Plane(int width = 5, int height = 5)
    {
        // generate the points along the y-axis
        double y = 0;
        for (double x = 0; x < width; x += 1) {
            for (double z = 0; z < height; z += 1) {
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
        ordered_ = true;
        orderedWidth_ = width;
        orderedHeight_ = height;

    }  // Constructor

};  // Plane
}  // namespace volcart::shapes
