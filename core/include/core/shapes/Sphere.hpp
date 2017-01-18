//
// Created by Melissa Shankle on 12/03/15.
// Design mostly taken from:
// http://blog.andreaskahler.com/2009/06/creating-icosphere-mesh-in-code.html
//
#pragma once

#include <string>
#include <unordered_map>

#include "core/shapes/ShapePrimitive.hpp"
#include "core/vc_defines.hpp"

namespace volcart
{
namespace shapes
{

class Sphere : public ShapePrimitive
{
public:
    explicit Sphere(float radius = 5, int recursionLevel = 2);

private:
    int midpoint_(int p1, int p2);

    std::unordered_map<std::string, int> indexCache_;

};  // Sphere
}
}
