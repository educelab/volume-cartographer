#pragma once

/** @file derivative.h
 *  @brief A small derivative library that handles calculating derivatives up to
 * second order
 *  @ingroup lrps
 */

#include <array>
#include <cassert>
#include <vector>

namespace volcart
{
namespace segmentation
{

// To be used later on when this is more parameterized
// clang-format off
/**
 * Array of common first derivative coefficients
 */
static std::array<std::array<double, 9>, 4> d1CentralDiffCoeffs = {
    0,     0,      0,    -1/2, 0, 1/2, 0,     0,     0,
    0,     0,      1/12, -2/3, 0, 2/3, -1/12, 0,     0,
    0,     -1/60,  3/20, -3/4, 0, 3/4, -3/20, 1/60,  0,
    1/280, -4/105, 1/5,  -4/5, 0, 4/5, -1/5,  4/105, -1/280
};

/**
 * Array of common second derivative coefficients
 */
static std::array<std::array<double, 9>, 4> d2CentralDiffCoeffs = {
    0,      0,     0,     1,   -2,      1,   0,     0,     0,
    0,      0,     -1/12, 4/3, -5/2,    4/3, -1/12, 0,     0,
    0,      1/90,  -3/20, 3/2, -49/18,  3/2, -3/20, 1/90,  0,
    -1/560, 8/315, -1/5,  8/5, -205/72, 8/5, -1/5,  8/315, -1/560
};
// clang-format on

/**
 * @fn T d1Forward(const std::vector<T>&vs, int32_t index, int32_t hstep=1)
 * @brief Returns the value of the next position after the index in the first
 * derivative array
 * @param vs Vector you want to move forward in
 * @param index Starting point in the vector
 * @param hstep How many elements you want to move by each time you move
 */
template <typename T>
T d1Forward(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(
        index >= 0 && index < int32_t(vs.size()) && "index not in range of vs");
    assert(index <= int32_t(vs.size()) && "index must not be last point");
    return (-vs[index] + vs[index + hstep]) / double(hstep);
}

/**
 * @fn T d1Backward(const std::vector<T>& vs, int32_t index, int32_t hstep=1)
 * @brief Returns the value in the position behind the index in the first
 * derivative array
 * @param hstep How many elements you want to move by each time you move
 * @param index Starting point in the vector
 * @param vs Vector you want to move in
 */
template <typename T>
T d1Backward(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(
        index >= 0 && index < int32_t(vs.size()) && "index not in range of vs");
    assert(index >= hstep && "index must not be first point");
    return (-vs[index - hstep] + vs[index]) / double(hstep);
}

/**
 * @fn T d1Central(const std::vector<T>&vs, int32_t index, int32_t hstep=1)
 * @brief Returns the position in between index and hstep
 * @param vs The vector that you want a value from
 * @param hstep How many elements to move each time you move in the vector
 * @param index Starting point in the vector
 */
template <typename T>
T d1Central(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(
        index >= 0 && index < int32_t(vs.size()) && "index not in range of vs");
    assert(index - hstep >= 0 && "index out of range");
    assert(index + hstep < int32_t(vs.size()) && "index out of range");
    return ((-0.5) * vs[index - hstep] + (0.5) * vs[index + hstep]) /
           double(hstep);
}

// from: https://en.wikipedia.org/wiki/Finite_difference_coefficient
// TODO: Implement more accurate derivatives
/**
 * @fn T d1FivePointStencil(const std::vector<T>& vs, int32_t index, int32_t
 * hstep = 1)
 * @brief Moves to the center of 5 points
 * @param vs Vector of possible coefficients
 * @param index Starting point in the vector
 * @param hstep Number of elements to move by each time you move
 * @see https://en.wikipedia.org/wiki/Finite_difference_coefficient
 * @return Current element
 */
template <typename T>
T d1FivePointStencil(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(
        index >= 0 && index < int32_t(vs.size()) && "index not in range of vs");
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

/**
 * @fn T d1At(const std::vector<T>&vs, int32_t index, int32_t hstep = 1)
 * @param vs Vector you wish to traverse
 * @param index Starting Point
 * @param hstep Number of elements to move by
 * @return The current location in the array
 */
template <typename T>
T d1At(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    if (index - hstep < 0) {
        return d1Forward(vs, index, hstep);
    } else if (index + hstep > int32_t(vs.size()) - 1) {
        return d1Backward(vs, index, hstep);
    } else if (
        index - hstep < hstep || index + hstep >= int32_t(vs.size()) - hstep) {
        return d1Central(vs, index, hstep);
    } else {
        return d1FivePointStencil(vs, index, hstep);
    }
}

/**
 * @fn std::vector<T> d1(const std::vector<T>& vs, int32_t hstep =1)
 * @brief Calculates the first derivative cofficents
 * @param vs Vector of derivative coffiecents
 * @param hstep Number of elements you want to move by
 * @return Vector of the first derivative coefficients
 */
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

/**
 * @fn T d2Forward(const std::vector<T>&vs, int32_t index, int32_t hstep=1)
 * @brief Returns the value of the next position after the index in the second
 * derivative array
 * @param vs Vector you want to move forward in
 * @param index Starting point in the vector
 * @param hstep How many elements you want to move by each time you move
 */
template <typename T>
T d2Forward(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(index >= 0 && index < int32_t(vs.size()) && "index out of range");
    assert(index + hstep < int32_t(vs.size()) && "index out of range");
    assert(index + 2 * hstep < int32_t(vs.size()) && "index out of  range");
    return (vs[index] + (-2.0) * vs[index + hstep] + vs[index + 2 * hstep]) /
           double(hstep * hstep);
}

/**
 * @fn T d2Backward(const std::vector<T>&vs, int32_t index, int32_t hstep=1)
 * @brief Returns the value in the position behind the index in the second
 * derivative array
 * @param hstep How many elements you want to move by each time you move
 * @param index Starting point in the vector
 * @param vs Vector you want to move in
 */
template <typename T>
T d2Backward(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(index >= 0 && index < int32_t(vs.size()) && "index out of range");
    assert(index - 2 * hstep >= 0 && "index out of range");
    assert(index - 1 * hstep >= 0 && "index out of range");
    return (vs[index - 2 * hstep] + (-2.0) * vs[index - hstep] + vs[index]) /
           double(hstep * hstep);
}

/**
 * @copydoc d1Central()
 */
template <typename T>
T d2Central(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(
        index >= 0 && index < int32_t(vs.size()) && "index not in range of vs");
    assert(index - hstep >= 0 && "index out of range");
    assert(index + hstep < int32_t(vs.size()) && "index out of range");
    return (vs[index - hstep] + (-2.0) * vs[index] + vs[index + hstep]) /
           double(hstep * hstep);
}

// from: https://en.wikipedia.org/wiki/Finite_difference_coefficient
// TODO: Implement more accurate derivatives

/** @copydoc d1FivePointStencil() */
template <typename T>
T d2FivePointStencil(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    assert(
        index >= 0 && index < int32_t(vs.size()) && "index not in range of vs");
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

/** @copydoc d1At() */
template <typename T>
T d2At(const std::vector<T>& vs, int32_t index, int32_t hstep = 1)
{
    if (index - hstep < 0) {
        return d2Forward(vs, index, hstep);
    } else if (index + hstep > int32_t(vs.size()) - 1) {
        return d2Backward(vs, index, hstep);
    } else if (
        index - hstep < hstep || index + hstep >= int32_t(vs.size()) - hstep) {
        return d2Central(vs, index, hstep);
    } else {
        return d2FivePointStencil(vs, index, hstep);
    }
}

/** @copydoc d1() */
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
