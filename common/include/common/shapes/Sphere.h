//
// Created by Melissa Shankle on 12/03/15.
// Design mostly taken from:
// http://blog.andreaskahler.com/2009/06/creating-icosphere-mesh-in-code.html
//
#pragma once

#include <math.h>

#include <opencv2/opencv.hpp>

#include "common/vc_defines.h"
#include "common/shapes/ShapePrimitive.h"

#include <unordered_map>

namespace volcart {
    namespace shapes {
        class Sphere : public ShapePrimitive {
        public:
            Sphere(float radius = 5, int recursionLevel = 2);

        private:
            int _midpoint( int p1, int p2 );

            std::unordered_map< std::string, int > _indexCache;

        }; // Sphere

    } // shapes
} // volcart
