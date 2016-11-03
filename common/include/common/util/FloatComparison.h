/**
  @file   FloatComparison.h
  @author Seth Parker
  @date   November 2016

  @brief  Methods for comparing floating point numbers.

  Provides templated methods for floating-point comparison. Based off the
  algorithms
  <a
  href="https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/">
  described by Bruce Dawson</a>.

  @ingroup Common
 */

#pragma once

#include <limits>
#include <type_traits>

namespace volcart
{

static constexpr double DefaultMaxDifference = 0.0000001;

/**
  @brief Compare if two floating-point numbers are "almost equal".

  Two numbers are "almost equal" if the absolute difference between the numbers
  is below some absolute error `epsMax` or if the absolute difference is
  smaller than `epsRel`% of the largest input parameter.

  @param lhs
  @param rhs
  @param epsMax Maximum absolute difference. Useful when comparing numbers
  close to 0.
  @param epsRel Maximum relative difference.
  @return `TRUE` or `FALSE`
 */
template <
    typename T,
    typename =
        typename std::enable_if<std::is_floating_point<T>::value, T>::type>
inline bool AlmostEqual(
    const T lhs,
    const T rhs,
    T epsMax = DefaultMaxDifference,
    T epsRel = std::numeric_limits<T>::epsilon())
{
    T d = std::fabs(lhs - rhs);
    if (d <= epsMax) {
        return true;
    }
    T l = std::fabs(lhs);
    T r = std::fabs(rhs);

    T largest = (r > l) ? r : l;
    if (d <= largest * epsRel) {
        return true;
    }

    return false;
};

}  // volcart
