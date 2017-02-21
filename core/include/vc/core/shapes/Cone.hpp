//
// Created by Melissa Shankle on 1/29/16.
//
#pragma once

#include <cmath>

#include <opencv2/core.hpp>

#include "ShapePrimitive.hpp"
#include "vc/core/vc_defines.hpp"

namespace volcart
{
namespace shapes
{
class Cone : public ShapePrimitive
{
public:
    Cone(int radius = 2, int height = 5, int recursion_level = 5)
    {

        // radius = radius of circle
        // height = used for height of circle
        // recursion_level = used to control how circular the cone will be
        //      level 0 will have 8 base points, level 1 will have 16 points,
        //      etc.

        std::vector<cv::Vec3d> circle_list;
        cv::Vec3d c_point;

        // add beginning 8 points of circle
        for (double l = 0.0; l < 2 * M_PI; l += (M_PI / 4)) {
            c_point[0] = radius * cos(l);
            c_point[1] = radius * sin(l);
            c_point[2] = height;
            circle_list.push_back(c_point);
        }

        int r = 0;

        while (r != recursion_level) {
            std::vector<cv::Vec3d> loop_list;
            loop_list = circle_list;
            circle_list.clear();
            double theta_inc =
                (2 * M_PI) /
                ((r + 2) *
                 8);  // number of section to calculate angle at each vertex
            double theta = 0.0;
            for (int p_id = 0; p_id < ((r + 2) * 8); p_id++) {

                c_point[0] =
                    radius * cos(theta);  // x value that will be on circle
                c_point[1] =
                    radius * sin(theta);  // y value that will be on circle
                c_point[2] = height;
                circle_list.push_back(c_point);

                theta += theta_inc;
            }
            r++;
        }

        // generate the starting vertices
        addVertex_(0, 0, 0);       // cone point
        addVertex_(0, 0, height);  // mid point of circle

        // generate the circle points
        for (auto point = circle_list.begin(); point != circle_list.end();
             ++point) {
            addVertex_((*point)[0], (*point)[1], (*point)[2]);
        }

        // generate the cells for faces
        // Our two "center" points are v_id 0 && 1, so start at 2
        int B;  // second vertex of each face
        for (size_t v_id = 2; v_id < points_.size(); ++v_id) {

            // Handle the last point in the circle
            if (v_id == points_.size() - 1) {
                B = 2;
            } else {
                B = v_id + 1;
            }

            addCell_(v_id, B, 0);
            addCell_(v_id, B, 1);
        }

        orderedPoints_ = false;
        orderedWidth_ = orderedHeight_ = 0;

    }  // Constructor

};  // Cone
}  // shapes
}  // volcart
