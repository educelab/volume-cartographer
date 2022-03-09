#pragma once

/**
 * @file   FloatComparison.hpp
 * @author Seth Parker
 * @date   November 2016
 *
 * @brief  Methods for comparing floating point numbers.
 *
 * Provides templated methods for floating-point comparison. Based off the
 * algorithms
 * <a
 * href="https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/">
 * described by Bruce Dawson</a>.
 *
 * @ingroup Util
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace volcart
{

static constexpr double DEFAULT_MAX_DIFFERENCE = 1e-7;

/**
 * @brief Compare if two floating-point numbers are "almost equal"
 *
 * Two numbers are "almost equal" if the absolute difference between the numbers
 * is below the absolute error `epsMax` or if the absolute difference is
 * smaller than `epsRel`% of the largest input parameter.
 *
 * @warning This method should not be assumed to be good for comparing against
 * zero. Depending on the circumstance, either an absolute or relative
 * difference might be preferable.
 *
 * @param lhs Left operand
 * @param rhs Right operand
 * @param epsAbs Maximum absolute difference. Useful when comparing numbers
 * close to 0.
 * @param epsRel Maximum relative difference.
 */
template <
    typename T,
    std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
inline auto AlmostEqual(
    const T lhs,
    const T rhs,
    T epsAbs = static_cast<T>(DEFAULT_MAX_DIFFERENCE),
    T epsRel = std::numeric_limits<T>::epsilon()) -> bool
{
    T d = std::abs(lhs - rhs);
    if (d <= epsAbs) {
        return true;
    }
    T l = std::abs(lhs);
    T r = std::abs(rhs);

    T largest = std::max(r, l);
    return d <= largest * epsRel;
}

}  // namespace volcart
