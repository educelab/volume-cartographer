#include "field.h"

// Constructor
DEMO::Field::Field(VolumePkg *v) {
    _volpkg = v;
    for (int i = 0; i < _volpkg->getNumberOfSlices(); ++i) {
        _field.push_back(v->getSliceAtIndex(i));
    }
}

// Trilinear Interpolation: Particles are not required
// to be at integer positions so we estimate their
// normals with their neighbors's known normals.
//
// formula from http://paulbourke.net/miscellaneous/interpolation/
uint16_t DEMO::Field::interpolate_at(cv::Vec3f point) {
    double int_part;
    double dx = modf(point(VC_INDEX_X), &int_part);
    double dy = modf(point(VC_INDEX_Y), &int_part);
    double dz = modf(point(VC_INDEX_Z), &int_part);

    int x_min = (int) point(VC_INDEX_X);
    int x_max = x_min + 1;
    int y_min = (int) point(VC_INDEX_Y);
    int y_max = y_min + 1;
    int z_min = (int) point(VC_INDEX_Z);
    int z_max = z_min + 1;

    // insert safety net
    if (x_min < 0 || y_min < 0 || z_min < 0 ||
        x_max >= _field[0].cols || y_max >= _field[0].rows ||
        z_max >= _field.size()) {
        return 0;
    }

    uint16_t vector = uint16_t(
        _field[z_min].at<uint16_t>(y_min, x_min) * (1 - dx) * (1 - dy) * (1 - dz) +
        _field[z_max].at<uint16_t>(y_min, x_min) * dx       * (1 - dy) * (1 - dz) +
        _field[z_min].at<uint16_t>(y_max, x_min) * (1 - dx) * dy       * (1 - dz) +
        _field[z_min].at<uint16_t>(y_min, x_max) * (1 - dx) * (1 - dy) * dz +
        _field[z_max].at<uint16_t>(y_min, x_max) * dx       * (1 - dy) * dz +
        _field[z_min].at<uint16_t>(y_max, x_max) * (1 - dx) * dy       * dz +
        _field[z_max].at<uint16_t>(y_max, x_min) * dx       * dy       * (1 - dz) +
        _field[z_max].at<uint16_t>(y_max, x_max) * dx       * dy       * dz);

    return vector;
}

// create a Slice from explicit component directions, height and width default to 64x64
Slice DEMO::Field::reslice(cv::Vec3f center, cv::Vec3f xv, cv::Vec3f yv, int32_t resliceHeight, int32_t resliceWidth) {
    cv::Vec3f xnorm = normalize(xv);
    cv::Vec3f ynorm = normalize(yv);

    cv::Mat m(resliceHeight, resliceWidth, CV_16UC1);
    cv::Vec3f origin = center - ((resliceWidth / 2) * xnorm + (resliceHeight / 2) * ynorm);

    for (int height = 0; height < resliceHeight; ++height) {
        for (int width = 0; width < resliceWidth; ++width) {
            cv::Vec3f v = origin + (height * ynorm) + (width * xnorm);
            m.at<uint16_t>(height, width) = this->interpolate_at(v);
        }
    }

    return Slice(m, origin, xnorm, ynorm);
}

// create a Slice from an axis of rotation and an angle
Slice DEMO::Field::resliceRadial(cv::Vec3f origin, cv::Vec3f rotation_axis, double theta, int32_t reslice_height,
                                 int32_t reslice_width) {
    cv::Vec3f slice_direcion(cos(theta), sin(theta), 0);
    slice_direcion =
            slice_direcion - (rotation_axis.dot(slice_direcion) / rotation_axis.dot(rotation_axis)) * rotation_axis;
    cv::Vec3f x_direction = normalize(slice_direcion);
    cv::Vec3f y_direction = normalize(rotation_axis);

    cv::Mat m(reslice_height, reslice_width, CV_16UC1);
    for (int height = 0; height < reslice_height; ++height) {
        for (int width = 0; width < reslice_width; ++width) {
            cv::Vec3f v = origin + (height * y_direction) + (width * x_direction);
            m.at<uint16_t>(height, width) = this->interpolate_at(v);
        }
    }

    return Slice(m, origin, x_direction, y_direction);
}
