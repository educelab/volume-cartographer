//
// Created by Melissa Shankle on 1/29/16.
//
#pragma once

#include <cmath>

#include "core/shapes/ShapePrimitive.h"
#include "core/vc_defines.h"

namespace volcart
{
namespace shapes
{
class Cube : public ShapePrimitive
{
public:
    Cube()
    {

        // generate the 8 vertices
        addVertex_(0, 0, 0);
        addVertex_(0, 5, 0);
        addVertex_(5, 5, 0);
        addVertex_(5, 0, 0);

        addVertex_(0, 0, 5);
        addVertex_(0, 5, 5);
        addVertex_(5, 5, 5);
        addVertex_(5, 0, 5);

        // generate the 12 cells for faces

        addCell_(0, 1, 2);
        addCell_(0, 1, 5);
        addCell_(0, 2, 3);
        addCell_(0, 3, 4);
        addCell_(0, 4, 5);
        addCell_(1, 2, 6);
        addCell_(1, 5, 6);
        addCell_(2, 3, 7);
        addCell_(2, 6, 7);
        addCell_(3, 4, 7);
        addCell_(4, 5, 7);
        addCell_(5, 6, 7);

        orderedPoints_ = false;
        orderedWidth_ = orderedHeight_ = 0;

    }  // Constructor

};  // Cube
}  // shapes
}  // volcart
