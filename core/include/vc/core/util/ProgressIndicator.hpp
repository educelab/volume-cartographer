#pragma once

#include <chrono>
#include <ostream>
#include <string>

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
class ConsoleProgressIndicator
{
    using Duration = std::chrono::duration<double>;
    using TimePoint =
        std::chrono::time_point<std::chrono::steady_clock, Duration>;

public:
    explicit ConsoleProgressIndicator(
        size_t iterations, std::string label = "");
    void update(size_t iteration);
    void complete();

private:
    size_t iterations_{0};
    size_t currentIteration_{0};
    TimePoint startTime_;
    TimePoint lastUpdate_;
    std::string label_;
    std::ostream& os_;
};
}  // namespace volcart
