//
// Created by Seth Parker on 11/19/15.
//

#ifndef VC_PLANE_H
#define VC_PLANE_H

#include "../vc_defines.h"
#include "ShapePrimitive.h"

namespace volcart {
    namespace shapes {
        class Plane : public ShapePrimitive {
        public:
            Plane() {
                // dimensions of the mesh plane
                int width = 5, height = 5;

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
            } // Constructor

        }; // Plane
    } // shapes
} // volcart

#endif //VC_PLANE_H