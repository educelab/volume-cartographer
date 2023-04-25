#include "vc/core/shapes/Spiral.hpp"

#include <cmath>

using namespace volcart;
using namespace volcart::shapes;

Spiral::Spiral(
    double arcLength,
    double height,
    double a,
    double b,
    double dTheta,
    double dH)
{
    auto s = (arcLength * b) / (a * std::sqrt(1 + (b * b)));
    auto angularDistance = std::log(s) / b;
    auto numCols = static_cast<int>(std::round(angularDistance / dTheta));
    auto numRows = static_cast<int>(std::round(height / dH));

    construct_spiral_(numCols, numRows, a, b, dTheta, dH);
}

Spiral::Spiral(
    double arcLength,
    double height,
    int numPts,
    int numCurves,
    double a,
    double b)
{
    auto s = (arcLength * b) / (a * std::sqrt(1 + (b * b)));
    auto angularDistance = std::log(s) / b;
    auto dTheta = angularDistance / numPts;
    auto dH = height / numCurves;

    construct_spiral_(numPts, numCurves, a, b, dTheta, dH);
}

void Spiral::construct_spiral_(
    int numCols, int numRows, double a, double b, double dTheta, double dH)
{
    // Construct a single spiral
    std::vector<cv::Vec2d> spiral;
    for (int i = 0; i < numCols; i++) {
        auto theta = dTheta * i;
        auto eTheta = std::exp(b * theta);
        auto x = a * std::cos(theta) * eTheta;
        auto y = a * std::sin(theta) * eTheta;

        spiral.emplace_back(x, y);
    }

    // Construct all of the spirals
    for (int i = 0; i < numRows; i++) {
        auto z = dH * i;
        for (const auto& p : spiral) {
            addVertex_(p[0], p[1], z);
        }
    }

    // generate the cells
    for (int i = 1; i < numRows; ++i) {
        for (int j = 1; j < numCols; ++j) {
            int v1, v2, v3, v4;
            v1 = i * numCols + j;
            v2 = v1 - 1;
            v3 = v2 - numCols;
            v4 = v1 - numCols;
            addCell_(v1, v2, v3);
            addCell_(v1, v3, v4);
        }
    }

    // Set this as an ordered mesh
    ordered_ = true;
    orderedWidth_ = numCols;
    orderedHeight_ = numRows;
}
