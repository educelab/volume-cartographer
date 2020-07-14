#pragma once

#include "vc/core/util/Signals.hpp"

namespace volcart
{

/** @brief Mixin type for classes which report their progress */
struct IterationsProgress {
    Signal<> progressStarted;
    Signal<size_t> progressUpdated;
    Signal<> progressComplete;
    virtual size_t progressIterations() const = 0;
};

}  // namespace volcart