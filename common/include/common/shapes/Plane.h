//
// Created by Seth Parker on 11/19/15.
//

#ifndef VC_PLANE_H
#define VC_PLANE_H

#include "common/vc_defines.h"
#include "common/shapes/ShapePrimitive.h"

namespace volcart {
    namespace shapes {
        class Plane : public ShapePrimitive {
        public:
            Plane(int width = 5, int height = 5) {

                // generate the points along the y-axis
                double y = 0;
                for ( double x = 0; x < width; ++x) {
                    for ( double z = 0; z < height; ++z ) {
                        _add_vertex(x, y, z);
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
                        _add_cell(v1, v2, v3);
                        _add_cell(v1, v3, v4);
                    }
                }

                // Set this as an ordered mesh
                _orderedPoints = true;
                _orderedWidth = width;
                _orderedHeight = height;

            } // Constructor

        }; // Plane
    } // shapes
} // volcart

#endif //VC_PLANE_H
