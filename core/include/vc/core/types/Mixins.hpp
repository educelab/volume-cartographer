#pragma once

/** @file */

#include <cstddef>

#include "vc/core/util/Signals.hpp"

namespace volcart
{

/** @brief Mixin type for classes which report their progress */
struct IterationsProgress {
    virtual ~IterationsProgress() = default;
    Signal<> progressStarted;
    Signal<std::size_t> progressUpdated;
    Signal<> progressComplete;
    virtual std::size_t progressIterations() const = 0;
};

}  // namespace volcart