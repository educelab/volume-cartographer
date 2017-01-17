#pragma once

#include <array>
#include <cassert>
#include <vector>

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
static constexpr std::array<std::array<double, 9>, 4> D1_CENTRAL_DIFF_COEFFS = {
    0,     0,      0,    -1/2, 0, 1/2, 0,     0,     0,
    0,     0,      1/12, -2/3, 0, 2/3, -1/12, 0,     0,
    0,     -1/60,  3/20, -3/4, 0, 3/4, -3/20, 1/60,  0,
    1/280, -4/105, 1/5,  -4/5, 0, 4/5, -1/5,  4/105, -1/280
};

static constexpr std::array<std::array<double, 9>, 4> D2_CENTRAL_DIFF_COEFFS = {
    0,      0,     0,     1,   -2,      1,   0,     0,     0,
    0,      0,     -1/12, 4/3, -5/2,    4/3, -1/12, 0,     0,
    0,      1/90,  -3/20, 3/2, -49/18,  3/2, -3/20, 1/90,  0,
    -1/560, 8/315, -1/5,  8/5, -205/72, 8/5, -1/5,  8/315, -1/560
};
// clang-format on

template <typename T>
T D1Forward(const std::vector<T>& vs, int index, int hstep = 1)
{
    assert(index >= 0 && index < int(vs.size()) && "index not in range of vs");
    assert(index <= int(vs.size()) && "index must not be last point");
    return (-vs[index] + vs[size_t(index) + hstep]) / double(hstep);
}

template <typename T>
T D1Backward(const std::vector<T>& vs, int index, int hstep = 1)
{
    assert(index >= 0 && index < int(vs.size()) && "index not in range of vs");
    assert(index >= hstep && "index must not be first point");
    return (-vs[index - hstep] + vs[index]) / double(hstep);
}

template <typename T>
T D1Central(const std::vector<T>& vs, int index, int hstep = 1)
{
    assert(index >= 0 && index < int(vs.size()) && "index not in range of vs");
    assert(index - hstep >= 0 && "index out of range");
    assert(index + hstep < int(vs.size()) && "index out of range");
    return ((-0.5) * vs[index - hstep] + (0.5) * vs[size_t(index) + hstep]) /
           double(hstep);
}

// from: https://en.wikipedia.org/wiki/Finite_difference_coefficient
// TODO(skarlage): Implement more accurate derivatives
template <typename T>
T D1FivePointStencil(const std::vector<T>& vs, int index, int hstep = 1)
{
    assert(index >= 0 && index < int(vs.size()) && "index not in range of vs");
    assert(index - 2 * hstep >= 0 && "index out of range\n");
    assert(index - 1 * hstep >= 0 && "index out of range\n");
    assert(index + 1 * hstep < int(vs.size()) && "index out of range\n");
    assert(index + 2 * hstep < int(vs.size()) && "index out of range\n");
    // clang-format off
    return (
        (1.0/12) * vs[index - 2 * hstep] +
        (-2.0/3) * vs[index - hstep] +
         (2.0/3) * vs[size_t(index) + hstep] +
       (-1.0/12) * vs[size_t(index) + 2 * hstep]) /
           double(hstep);
    // clang-format on
}

template <typename T>
T D1At(const std::vector<T>& vs, int index, int hstep = 1)
{
    if (index - hstep < 0) {
        return D1Forward(vs, index, hstep);
    } else if (index + hstep > int(vs.size()) - 1) {
        return D1Backward(vs, index, hstep);
    } else if (
        index - hstep < hstep || index + hstep >= int(vs.size()) - hstep) {
        return D1Central(vs, index, hstep);
    } else {
        return D1FivePointStencil(vs, index, hstep);
    }
}

template <typename T>
std::vector<T> D1(const std::vector<T>& vs, int hstep = 1)
{
    std::vector<T> dvs;
    dvs.reserve(vs.size());
    for (size_t i = 0; i < vs.size(); ++i) {
        dvs.push_back(D1At(vs, i, hstep));
    }
    return dvs;
}

template <typename T>
T D2Forward(const std::vector<T>& vs, int index, int hstep = 1)
{
    assert(index >= 0 && index < int(vs.size()) && "index out of range");
    assert(index + hstep < int(vs.size()) && "index out of range");
    assert(index + 2 * hstep < int(vs.size()) && "index out of  range");
    return (vs[index] + (-2.0) * vs[size_t(index) + hstep] +
            vs[size_t(index) + 2 * hstep]) /
           double(hstep * hstep);
}

template <typename T>
T D2Backward(const std::vector<T>& vs, int index, int hstep = 1)
{
    assert(index >= 0 && index < int(vs.size()) && "index out of range");
    assert(index - 2 * hstep >= 0 && "index out of range");
    assert(index - 1 * hstep >= 0 && "index out of range");
    return (vs[index - 2 * hstep] + (-2.0) * vs[index - hstep] + vs[index]) /
           double(hstep * hstep);
}

template <typename T>
T D2Central(const std::vector<T>& vs, int index, int hstep = 1)
{
    assert(index >= 0 && index < int(vs.size()) && "index not in range of vs");
    assert(index - hstep >= 0 && "index out of range");
    assert(index + hstep < int(vs.size()) && "index out of range");
    return (vs[index - hstep] + (-2.0) * vs[index] +
            vs[size_t(index) + hstep]) /
           double(hstep * hstep);
}

// from: https://en.wikipedia.org/wiki/Finite_difference_coefficient
// TODO(skarlage): Implement more accurate derivatives
template <typename T>
T D2FivePointStencil(const std::vector<T>& vs, int index, int hstep = 1)
{
    assert(index >= 0 && index < int(vs.size()) && "index not in range of vs");
    assert(index - 2 * hstep >= 0 && "index out of range");
    assert(index - 1 * hstep >= 0 && "index out of range");
    assert(index + 1 * hstep < int(vs.size()) && "index out of range");
    assert(index + 2 * hstep < int(vs.size()) && "index out of range");
    // clang-format off
    return (
       (-1.0/12) * vs[index - 2 * hstep] +
         (4.0/3) * vs[index - hstep] +
        (-5.0/2) * vs[index] + 
         (4.0/3) * vs[size_t(index) + hstep] +
       (-1.0/12) * vs[size_t(index) + 2 * hstep]) /
           double(hstep * hstep);
    // clang-format on
}

template <typename T>
T D2At(const std::vector<T>& vs, int index, int hstep = 1)
{
    if (index - hstep < 0) {
        return D2Forward(vs, index, hstep);
    } else if (index + hstep > int(vs.size()) - 1) {
        return D2Backward(vs, index, hstep);
    } else if (
        index - hstep < hstep || index + hstep >= int(vs.size()) - hstep) {
        return D2Central(vs, index, hstep);
    } else {
        return D2FivePointStencil(vs, index, hstep);
    }
}

template <typename T>
std::vector<T> D2(const std::vector<T>& vs, int hstep = 1)
{
    std::vector<T> dvs;
    dvs.reserve(vs.size());
    for (size_t i = 0; i < vs.size(); ++i) {
        dvs.push_back(D2At(vs, i, hstep));
    }
    return dvs;
}
}
}
