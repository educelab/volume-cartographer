//
// Created by Melissa Shankle on 1/29/16.
//

#ifndef VC_CUBE_H
#define VC_CUBE_H

#include <math.h>

#include <opencv2/opencv.hpp>

#include "common/vc_defines.h"
#include "ShapePrimitive.h"

namespace volcart {
    namespace shapes {
        class Cube : public ShapePrimitive {
        public:
            Cube() {

                // generate the 8 vertices
                _add_vertex( 0, 0, 0);
                _add_vertex( 0, 5, 0);
                _add_vertex( 5, 5, 0);
                _add_vertex( 5, 0, 0);

                _add_vertex( 0, 0, 5);
                _add_vertex( 0, 5, 5);
                _add_vertex( 5, 5, 5);
                _add_vertex( 5, 0, 5);

                // generate the 12 cells for faces

                _add_cell(0, 1, 2);
                _add_cell(0, 1, 5);
                _add_cell(0, 2, 3);
                _add_cell(0, 3, 4);
                _add_cell(0, 4, 5);
                _add_cell(1, 2, 6);
                _add_cell(1, 5, 6);
                _add_cell(2, 3, 7);
                _add_cell(2, 6, 7);
                _add_cell(3, 4, 7);
                _add_cell(4, 5, 7);
                _add_cell(5, 6, 7);

                _orderedPoints = false;
            } // Constructor


        }; // Cube
    } // shapes
} // volcart

#endif //VC_CUBE_H
