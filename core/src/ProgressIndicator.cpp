#include "vc/core/util/ProgressIndicator.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>

#include "vc/core/util/DateTime.hpp"

using namespace volcart;

ConsoleProgressIndicator::ConsoleProgressIndicator(
    size_t iterations, std::string label)
    : iterations_{iterations}, label_{std::move(label)}, os_{std::cout}
{
    startTime_ = std::chrono::steady_clock::now();
}

void ConsoleProgressIndicator::update(size_t iteration)
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

void ConsoleProgressIndicator::complete()
{
    update(iterations_);
    os_ << std::endl;
}