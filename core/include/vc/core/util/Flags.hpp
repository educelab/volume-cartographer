#pragma once

/**
 * @file
 *
 * @brief Helper functions for bitmask flags
 *
 * Assumes that the flag type implements the BitmaskType named requirement and
 * that the unset flag value is 0.
 */

namespace volcart::flag
{

/** @brief Returns whether all provided flags are set on the value */
template <typename T, typename... Ts>
auto is_set(const T& value, const T& flag, Ts&&... flags) -> bool
{
    auto res = (value & flag) == flag;
    ((res &= (value & flags) == flags), ...);
    return res;
}

/** @brief Returns whether any flag is set on the value */
template <typename T>
auto is_set(const T& value) -> bool
{
    return value != T{0};
}

/** @brief Sets all given flags on the value */
template <typename T, typename... Ts>
void set(T& value, const T& flag, Ts&&... flags)
{
    value |= flag;
    ((value |= flags), ...);
}

/** @brief Removes all given flags from the value */
template <typename T, typename... Ts>
void unset(T& value, const T& flag, Ts&&... flags)
{
    value &= ~flag;
    ((value &= ~flags), ...);
}

/** @brief Removes all flags from the value */
template <typename T>
void unset(T& value)
{
    value = 0;
}

/** @brief Flips all given flags on the value */
template <typename T, typename... Ts>
void flip(T& value, const T& flag, Ts&&... flags)
{
    value ^= flag;
    ((value ^= flags), ...);
}

}  // namespace volcart::flag
