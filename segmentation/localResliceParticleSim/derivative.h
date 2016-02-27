#pragma once

#ifndef VOLCART_SEGMENTATION_DERIVATIVE_H
#define VOLCART_SEGMENTATION_DERIVATIVE_H

#include <vector>
#include <array>
#include <cassert>

namespace volcart
{
namespace segmentation
{
/*
 * A small derivative library that handles calculating derivatives up to second
 * order
 */

// To be used later on when this is more parameterized
// clang-format off
static std::array<std::array<double, 9>, 4> d1CentralDiffCoeffs = {
    0,     0,      0,    -1/2, 0, 1/2, 0,     0,     0,
    0,     0,      1/12, -2/3, 0, 2/3, -1/12, 0,     0,
    0,     -1/60,  3/20, -3/4, 0, 3/4, -3/20, 1/60,  0,
    1/280, -4/105, 1/5,  -4/5, 0, 4/5, -1/5,  4/105, -1/280
};

static std::array<std::array<double, 9>, 4> d2CentralDiffCoeffs = {
    0,      0,     0,     1,   -2,      1,   0,     0,     0,
    0,      0,     -1/12, 4/3, -5/2,    4/3, -1/12, 0,     0,
    0,      1/90,  -3/20, 3/2, -49/18,  3/2, -3/20, 1/90,  0,
    -1/560, 8/315, -1/5,  8/5, -205/72, 8/5, -1/5,  8/315, -1/560
};
// clang-format on

template <typename T>
T d1Forward(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(index >= 0 && index < int32_t(vs.size()) &&
           "index not in range of vs");
    assert(index <= int32_t(vs.size()) && "index must not be last point");
    return (-vs[index] + vs[index + hstep]) / double(hstep);
}

template <typename T>
T d1Backward(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(index >= 0 && index < int32_t(vs.size()) &&
           "index not in range of vs");
    assert(index >= hstep && "index must not be first point");
    return (-vs[index - hstep] + vs[index]) / double(hstep);
}

template <typename T>
T d1Central(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(index >= 0 && index < int32_t(vs.size()) &&
           "index not in range of vs");
    assert(index - hstep >= 0 && "index out of range");
    assert(index + hstep < int32_t(vs.size()) && "index out of range");
    return ((-0.5) * vs[index - hstep] + (0.5) * vs[index + hstep]) /
           double(hstep);
}

// from: https://en.wikipedia.org/wiki/Finite_difference_coefficient
// TODO: Implement more accurate derivatives
template <typename T>
T d1FivePointStencil(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(index >= 0 && index < int32_t(vs.size()) &&
           "index not in range of vs");
    assert(index - 2 * hstep >= 0 && "index out of range\n");
    assert(index - 1 * hstep >= 0 && "index out of range\n");
    assert(index + 1 * hstep < int32_t(vs.size()) && "index out of range\n");
    assert(index + 2 * hstep < int32_t(vs.size()) && "index out of range\n");
    // clang-format off
    return (
        (1.0/12) * vs[index - 2 * hstep] +
        (-2.0/3) * vs[index - hstep] +
         (2.0/3) * vs[index + hstep] +
       (-1.0/12) * vs[index + 2 * hstep]) /
           double(hstep);
    // clang-format on
}

template <typename T>
T d1At(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    if (index - hstep <= 0) {
        return d1Forward(vs, index, hstep);
    } else if (index + hstep >= int32_t(vs.size()) - 1) {
        return d1Backward(vs, index, hstep);
    } else if (index - hstep < hstep ||
               index + hstep >= int32_t(vs.size()) - hstep) {
        return d1Central(vs, index, hstep);
    } else {
        return d1FivePointStencil(vs, index, hstep);
    }
}

template <typename T>
std::vector<T> d1(const std::vector<T>& vs, int32_t hstep = 1)
{
    std::vector<T> dvs;
    dvs.reserve(vs.size());
    for (size_t i = 0; i < vs.size(); ++i) {
        dvs.push_back(d1At(vs, i, hstep));
    }
    return dvs;
}

template <typename T>
T d2Forward(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(index >= 0 && index < int32_t(vs.size()) && "index out of range");
    assert(index + hstep < int32_t(vs.size()) && "index out of range");
    assert(index + 2 * hstep < int32_t(vs.size()) && "index out of  range");
    return (vs[index] + (-2.0) * vs[index + hstep] + vs[index + 2 * hstep]) /
           double(hstep * hstep);
}

template <typename T>
T d2Backward(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(index >= 0 && index < int32_t(vs.size()) && "index out of range");
    assert(index - 2 * hstep >= 0 && "index out of range");
    assert(index - 1 * hstep >= 0 && "index out of range");
    return (vs[index - 2 * hstep] + (-2.0) * vs[index - hstep] + vs[index]) /
           double(hstep * hstep);
}

template <typename T>
T d2Central(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(index >= 0 && index < int32_t(vs.size()) &&
           "index not in range of vs");
    assert(index - hstep >= 0 && "index out of range");
    assert(index + hstep < int32_t(vs.size()) && "index out of range");
    return (vs[index - hstep] + (-2.0) * vs[index] + vs[index + hstep]) /
           double(hstep * hstep);
}

// from: https://en.wikipedia.org/wiki/Finite_difference_coefficient
// TODO: Implement more accurate derivatives
template <typename T>
T d2FivePointStencil(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(index >= 0 && index < int32_t(vs.size()) &&
           "index not in range of vs");
    assert(index - 2 * hstep >= 0 && "index out of range");
    assert(index - 1 * hstep >= 0 && "index out of range");
    assert(index + 1 * hstep < int32_t(vs.size()) && "index out of range");
    assert(index + 2 * hstep < int32_t(vs.size()) && "index out of range");
    // clang-format off
    return (
       (-1.0/12) * vs[index - 2 * hstep] +
         (4.0/3) * vs[index - hstep] +
        (-5.0/2) * vs[index] + 
         (4.0/3) * vs[index + hstep] +
       (-1.0/12) * vs[index + 2 * hstep]) /
           double(hstep * hstep);
    // clang-format on
}

template <typename T>
T d2At(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    if (index - hstep <= 0) {
        return d2Forward(vs, index, hstep);
    } else if (index + hstep >= int32_t(vs.size()) - 1) {
        return d2Backward(vs, index, hstep);
    } else if (index - hstep < hstep ||
               index + hstep >= int32_t(vs.size()) - hstep) {
        return d2Central(vs, index, hstep);
    } else {
        return d2FivePointStencil(vs, index, hstep);
    }
}

template <typename T>
std::vector<T> d2(const std::vector<T>& vs, int32_t hstep = 1)
{
    std::vector<T> dvs;
    dvs.reserve(vs.size());
    for (size_t i = 0; i < vs.size(); ++i) {
        dvs.push_back(d2At(vs, i, hstep));
    }
    return dvs;
}
}
}

#endif
