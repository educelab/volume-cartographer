//
// Created by Seth Parker on 6/10/16.
//

#ifndef VC_ABF_MATH_H
#define VC_ABF_MATH_H

#include <opencv2/opencv.hpp>

#include "vc_defines.h"

// Returns the angle between AB & AC
double vec_angle( volcart::QuadPoint A, volcart::QuadPoint B, volcart::QuadPoint C );

#endif //VC_ABF_MATH_H
