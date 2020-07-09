#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>

#include "vc/core/util/DateTime.hpp"

namespace volcart
{
/**
 * @brief Console progress indicator
 *
 * Prints the number of iterations completed out of the total, and displays
 * elapsed time along with estimated total time.
 *
 * This progress indicator is purely cosmetic and is not a substitute for
 * proper event logging. This class does not gracefully handle other text being
 * printed to the console while it is operating. As such, this class should be
 * used sparingly.
 *
 * In the future, this will likely be split into a core progress signaller and
 * both console and GUI indicators.
 *
 * @ingroup Util
 */
template <typename CounterType>
class ConsoleProgressIndicator
{
    using Duration = std::chrono::duration<double>;
    using TimePoint =
        std::chrono::time_point<std::chrono::steady_clock, Duration>;
public:
    explicit ConsoleProgressIndicator(
        CounterType iterations, std::string label = "")
        : iterations_{iterations}, label_{std::move(label)}, os_{std::cout}
    {
        startTime_ = std::chrono::steady_clock::now();
    }

    void update(CounterType iteration)
    {
        // Update the current iteration
        currentIteration_ = iteration;

        // Get the time difference between updates
        auto now = std::chrono::steady_clock::now();
        auto timeDiff = (now - lastUpdate_).count();

        // Skip if we've started iterating and the time diff is too small
        if (currentIteration_ > 0 && timeDiff <= 0.1) {
            return;
        }

        // Append the iteration progress to the output
        std::stringstream ss;
        ss << label_ << " " << currentIteration_ << "/" << iterations_;

        // Append the remaining time to the output
        if (currentIteration_ > 0) {
            auto perc = static_cast<double>(currentIteration_) / iterations_;
            auto elapsed = now - startTime_;
            auto estimated = (now - startTime_) / perc;
            auto eta = estimated - elapsed;
            ss << " [" << DurationToDurationString(elapsed);
            ss << "<" << DurationToDurationString(eta) << "]";
        }
        os_ << std::setw(50) << std::left << ss.str() << "\r";
        os_ << std::flush;
        lastUpdate_ = now;
    }

    void complete()
    {
        update(iterations_);
        os_ << std::endl;
    }

private:
    CounterType iterations_{0};
    CounterType currentIteration_{0};
    TimePoint startTime_;
    TimePoint lastUpdate_;
    std::string label_;
    std::ostream& os_;
};

using DiscreteConsoleProgressIndicator = ConsoleProgressIndicator<size_t>;
using PercentConsoleProgressIndicator = ConsoleProgressIndicator<float>;
}  // namespace volcart
