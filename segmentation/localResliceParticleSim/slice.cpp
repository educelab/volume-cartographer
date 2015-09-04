#include "slice.h"

// color defines
// mostly the same as what's in structureTensorParticleSim.cpp
// both should be moved somewhere more global
#define WHITE 255
#define BLACK 0

#define BGR_BLUE cv::Scalar(255, 0, 0)
#define BGR_GREEN cv::Scalar(0, 255, 0)
#define BGR_RED cv::Scalar(0, 0, 255)

#define BGR_CYAN cv::Scalar(255, 255, 0)
#define BGR_YELLOW cv::Scalar(0, 255, 255)
#define BGR_MAGENTA cv::Scalar(255, 0, 255)

#define BGR_WHITE cv::Scalar(255, 255, 255)

#define DEBUG_ARROW_SCALAR 20

// basic constructor
Slice::Slice(cv::Mat slice, cv::Vec3f origin, cv::Vec3f x_direction, cv::Vec3f y_direction) {
    slice_ = slice;
    origin_ = origin;
    x_component_ = x_direction;
    y_component_ = y_direction;
}

cv::Vec3f Slice::findNextPosition() {
    cv::Mat analysis = this->analyze();
    cv::Point center(analysis.cols / 2, analysis.rows / 2);

    // find the new position in the reslice
    cv::Point offset(0, 1);
    for (int xoffset = 0; xoffset < analysis.cols / 2; ++xoffset) {
        cv::Point positive_offset(xoffset, 1);
        if (analysis.at<uchar>(center + positive_offset)) {
            offset = positive_offset;
            break;
        }

        cv::Point negative_offset(-xoffset, 1);
        if (analysis.at<uchar>(center + negative_offset)) {
            offset = negative_offset;
            break;
        }
    }
    center += offset;

    // map the pixel coordinate back into 3d space
    // we may want to make this its own method
    cv::Vec3f xyzPosition = origin_ + (center.x * x_component_ + center.y * y_component_);

    return xyzPosition;
}

void Slice::debugDraw(int debugDrawOptions) {
    cv::Mat debug = slice_.clone();
    debug *= 1. / 255;
    debug.convertTo(debug, CV_8UC3);
    cvtColor(debug, debug, CV_GRAY2BGR);


    // project xyz coordinate reference onto viewing plane with the formula
    //
    // [x_component_] [x]
    // [y_component_] [y]
    //                [z]
    //
    // which becomes componentwise pairs (x_1, x_2) (y_1, y_2) (z_1, z_2) when we only care about i, j, and k
    if (debugDrawOptions & DEBUG_DRAW_XYZ) {
        cv::Point x_arrow_offset(DEBUG_ARROW_SCALAR * x_component_(VC_INDEX_X),
                                 DEBUG_ARROW_SCALAR * y_component_(VC_INDEX_X));
        cv::Point y_arrow_offset(DEBUG_ARROW_SCALAR * x_component_(VC_INDEX_Y),
                                 DEBUG_ARROW_SCALAR * y_component_(VC_INDEX_Y));
        cv::Point z_arrow_offset(DEBUG_ARROW_SCALAR * x_component_(VC_INDEX_Z),
                                 DEBUG_ARROW_SCALAR * y_component_(VC_INDEX_Z));

        cv::Point coordinate_origin(DEBUG_ARROW_SCALAR, DEBUG_ARROW_SCALAR);
        rectangle(debug, cv::Point(0, 0), 2 * cv::Point(DEBUG_ARROW_SCALAR, DEBUG_ARROW_SCALAR), BGR_WHITE);
        arrowedLine(debug, coordinate_origin, coordinate_origin + x_arrow_offset, BGR_RED);
        arrowedLine(debug, coordinate_origin, coordinate_origin + y_arrow_offset, BGR_GREEN);
        arrowedLine(debug, coordinate_origin, coordinate_origin + z_arrow_offset, BGR_BLUE);
    }

    if (debugDrawOptions & DEBUG_DRAW_CORNER_COORDINATES) {
        std::stringstream trc;
        cv::Vec3f top_right_corner = origin_ + debug.cols * x_component_;
        trc << "(" << (int) top_right_corner(VC_INDEX_X)
        << "," << (int) top_right_corner(VC_INDEX_Y)
        << "," << (int) top_right_corner(VC_INDEX_Z) << ")";

        std::stringstream blc;
        cv::Vec3f bottom_left_corner = origin_ + debug.rows * y_component_;
        blc << "(" << (int) bottom_left_corner(VC_INDEX_X)
        << "," << (int) bottom_left_corner(VC_INDEX_Y)
        << "," << (int) bottom_left_corner(VC_INDEX_Z) << ")";

        putText(debug, trc.str(), cv::Point(debug.cols - 125, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, BGR_WHITE);
        putText(debug, blc.str(), cv::Point(5, debug.rows - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, BGR_WHITE);
    }

    if (debugDrawOptions & DEBUG_DRAW_CENTER) {
        cv::Point imcenter(debug.cols / 2, debug.rows / 2);
        arrowedLine(debug, imcenter, imcenter + cv::Point(debug.cols / 2 - 1, 0), BGR_YELLOW);
        circle(debug, imcenter, 2, BGR_MAGENTA, -1);
    }

    namedWindow("DEBUG DRAW", cv::WINDOW_AUTOSIZE);
    imshow("DEBUG DRAW", debug);
}

void Slice::debugAnalysis() {
    cv::Mat analysis = this->analyze();
    namedWindow("DEBUG ANALYSIS", cv::WINDOW_AUTOSIZE);
    imshow("DEBUG ANALYSIS", analysis);
}

cv::Mat Slice::mat() {
    return slice_.clone();
}


// function for analyzing the slice
// returns a cv::Mat for viewing in a debug window
// or finding out where to move next
cv::Mat Slice::analyze() {
    // second derivative gives us a nice void space
    // around the pages of the scroll
    cv::Mat temp_slice = slice_.clone();
    cv::Mat grad_x;
    GaussianBlur(temp_slice, grad_x, cv::Size(3, 3), 0);
    Sobel(grad_x, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
    Sobel(grad_x, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);

    cv::Mat fill;
    grad_x *= 1. / 255;
    grad_x.convertTo(fill, CV_8UC1);

    // slice is easier to work with when it's just black and white
    cv::Point center(fill.cols / 2, fill.rows / 2);
    floodFill(fill, center, WHITE);
    threshold(fill, fill, WHITE - 1, WHITE, cv::THRESH_BINARY);

    // skeletonize the slice
    // there might be a way to get the same effect with an opencv erode
    for (int y = 0; y < fill.rows; ++y) {
        int xindex = 0;

        // don't start on a white cell
        while (fill.at<uint8_t>(cv::Point(xindex, y)) == WHITE) {
            xindex++;
        }

        while (xindex < fill.cols) {
            int left_side;
            int right_side;

            // only used for skipping bound_invalid
            goto bound_valid;

            // go the the next row in the slice
            bound_invalid:
            break;

            bound_valid:
            // find the left side of a white region
            while (fill.at<uint8_t>(cv::Point(xindex, y)) != WHITE) {
                xindex++;
                if (xindex >= fill.cols)
                    goto bound_invalid;
            }
            left_side = xindex;

            // find the right side of a white region
            while (fill.at<uint8_t>(cv::Point(xindex + 1, y)) != BLACK) {
                xindex++;
                if (xindex >= fill.cols)
                    goto bound_invalid;
            }
            right_side = xindex;

            // black out everything in the region
            // except the center
            for (int i = left_side; i <= right_side; ++i) {
                fill.at<uint8_t>(cv::Point(i, y)) = BLACK;
            }
            fill.at<uint8_t>(cv::Point((right_side + left_side) / 2, y)) = WHITE;

            // putting in two points might be useful when
            // there isn't an integer valued center
            // if ((left_side + right_side) % 2 == 1)
            //   fill.at<unsigned char>(cv::Point((right_side + left_side + 1) / 2, y)) = WHITE;

            // we can get stuck if we don't do another increment
            xindex++;
        }
    }

    return fill;
}
