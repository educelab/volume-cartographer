#pragma once

/** @file */

#include <chrono>
#include <iterator>
#include <memory>

#include <indicators/cursor_control.hpp>
#include <indicators/progress_bar.hpp>

namespace volcart
{

/**
 * Make a new ProgressBar with project defined default formatting
 *
 * @ingroup Support
 */
inline auto NewProgressBar(size_t maxProg, std::string label = "")
{
    using namespace indicators;
    namespace opt = option;
    // clang-format off
    return std::make_shared<indicators::ProgressBar>(
        opt::BarWidth{22},
        opt::PrefixText{std::move(label)},
        opt::MaxProgress{maxProg},
        opt::ShowPercentage{true},
        opt::ShowElapsedTime{true},
        opt::ShowRemainingTime{true},
        opt::Start{" ["},
        opt::Fill{"■"},
        opt::Lead{"■"},
        opt::Remainder{" "},
        opt::End{"]"});
    // clang-format on
}

/**
 * Configuration options for ReportProgress
 *
 * @ingroup Support
 */
struct ProgressConfig {
    /** Descriptive label */
    std::string label;
    /** If true, show iterations as postfix (e.g. 25/256) */
    bool showIters{true};
    /** If true, use pretty color printing */
    bool useColors{false};
    /**
     * Reporting interval. The logger will only print when the duration between
     * the last reported update and the current update exceeds this interval.
     */
    std::chrono::milliseconds interval{std::chrono::milliseconds{1000}};
};

template <class Iterable>
inline auto ProgressWrap(Iterable&& it, ProgressConfig cfg = {});

/**
 * Iterable wrapper for tracking iteration progress. This class's iterator meets
 * the LegacyIterator requirements. Each call to `begin()` initializes a new
 * indicators::ProgressBar on the heap. Incrementing the returned iterator
 * advances the progress bar by one iteration. Copies of the iterator will
 * share the same progress bar instance.
 *
 * Use volcart::ProgressWrap, which provides a convenient method
 * for creating a ProgressBarIterable for range-based for loops.
 *
 * @ingroup Support
 */
template <class Iterable>
class ProgressBarIterable
{
private:
    /**
     * Custom iterator class. Does all of the progress tracking work. Stores an
     * iterator to a container and a progress bar instance. Incrementing this
     * class increments both the wrapped iterator and the progress bar.
     */
    template <class T>
    class ConsoleProgressIterator
    {
    private:
        /** Wrapped iterator */
        T it_;
        /** Shared progress bar instance */
        std::shared_ptr<indicators::ProgressBar> bar_;
        /** Current progress value */
        size_t prog_{0};
        /** Max progress value */
        size_t maxProg_{0};
        /** Use colors */
        bool useColors_{false};
        /** Last update timestamp */
        std::chrono::steady_clock::time_point lastUpdate_;
        /** Update interval */
        std::chrono::steady_clock::duration interval_{
            std::chrono::milliseconds(500)};

    public:
        /** @{ Iterator type traits */
        using difference_type = size_t;
        using value_type = typename std::iterator_traits<T>::value_type;
        using pointer = typename std::iterator_traits<T>::pointer;
        using reference = typename std::iterator_traits<T>::reference;
        using iterator_category = std::input_iterator_tag;
        /** @} */

        /** Constructor for the begin() iterator */
        explicit ConsoleProgressIterator(
            T beginIt,
            size_t max,
            std::string label,
            bool useColors,
            std::chrono::steady_clock::duration interval =
                std::chrono::seconds(1))
            : it_{beginIt}
            , maxProg_{max}
            , useColors_{useColors}
            , interval_{interval}
        {
            bar_ = NewProgressBar(maxProg_, std::move(label));
            if (useColors_) {
                using indicators::Color;
                using indicators::option::ForegroundColor;
                bar_->set_option(ForegroundColor{Color::yellow});
            }
        }

        /** Constructor for the end() iterator */
        explicit ConsoleProgressIterator(T endIt) : it_{endIt} {}

        /** Destructor */
        ~ConsoleProgressIterator()
        {
            if (useColors_) {
                indicators::show_console_cursor(true);
            }
        }

        /** Get the underlying referenced object */
        auto operator*() const -> reference { return *it_; }

        /** Equality comparison: defer to wrapped iterators */
        auto operator==(const ConsoleProgressIterator& other) const -> bool
        {
            return it_ == other.it_;
        }

        /** Inequality comparison */
        auto operator!=(const ConsoleProgressIterator& other) const -> bool
        {
            return !(*this == other);
        }

        /** Increment the wrapped iterator and progress bar */
        auto operator++() -> ConsoleProgressIterator&
        {
            // Increment iterator
            ++it_;
            // Increment progress counter
            ++prog_;

            // Update the progress bar
            using indicators::Color;
            using indicators::option::ForegroundColor;
            using indicators::option::PostfixText;
            auto post = std::to_string(prog_) + "/" + std::to_string(maxProg_);
            bar_->set_option(PostfixText{post});
            if (prog_ < maxProg_) {
                auto now = std::chrono::steady_clock::now();
                if (now - lastUpdate_ > interval_) {
                    if (useColors_) {
                        indicators::show_console_cursor(false);
                    }
                    bar_->set_progress(prog_);
                    lastUpdate_ = now;
                }
            } else {
                if (useColors_) {
                    bar_->set_option(ForegroundColor{Color::green});
                }
                bar_->tick();
                if (useColors_) {
                    indicators::show_console_cursor(true);
                }
            }
            return *this;
        }
    };

    using IteratorType = decltype(std::begin(std::declval<Iterable&>()));
    ProgressBarIterable(Iterable&& container, ProgressConfig cfg)
        : container_{std::forward<Iterable>(container)}, cfg_{std::move(cfg)}
    {
    }

    friend auto ProgressWrap<Iterable>(Iterable&& it, ProgressConfig cfg);

public:
    using iterator = ConsoleProgressIterator<IteratorType>;
    using const_iterator = ConsoleProgressIterator<const IteratorType>;

    /** Destructor */
    ~ProgressBarIterable()
    {
        if (cfg_.useColors) {
            indicators::show_console_cursor(true);
        }
    }

    /** Get a new ProgressIterable::iterator. Instantiates a new progress bar.
     */
    auto begin() const -> iterator
    {
        auto begin = std::begin(container_);
        auto end = std::end(container_);
        return iterator{
            begin, static_cast<size_t>(std::distance(begin, end)), cfg_.label,
            cfg_.useColors};
    }

    /**
     * Get the end-valued ProgressIterable::iterator. Does not instantiate a
     * new progress bar.
     */
    auto end() const -> iterator { return iterator{std::end(container_)}; }

    /**
     * Get a new ProgressIterable::const_iterator. Instantiates a new progress
     * bar.
     */
    auto cbegin() const -> const_iterator
    {
        auto begin = std::begin(container_);
        auto end = std::end(container_);
        return const_iterator{
            begin, std::distance(begin, end), cfg_.label, cfg_.useColors};
    }

    /**
     * Get the end-valued ProgressIterable::const_iterator. Does not instantiate
     * a new progress bar.
     */
    auto cend() const -> const_iterator
    {
        return const_iterator{std::end(container_)};
    }

    /** Get the size of the underlying iterable */
    [[nodiscard]] auto size() const -> std::size_t
    {
        return std::size(container_);
    }

private:
    /** Wrapped container */
    Iterable container_;
    /** Progress bar config */
    ProgressConfig cfg_;
};

/**
 * @brief Wrap a progress bar around an Iterable object.
 *
 * Returns a ProgressBarIterable which wraps the provided Iterable object.
 *
 * @code
 *
 * using Vector = std::vector<int>;
 *
 * // Using ConsoleProgressBar
 * Vector integers{0, 1, 2};
 * for(auto& i : ProgressWrap(integers)) {
 *     i += 1;
 * }
 * // integers == {1, 2, 3}
 *
 * @endcode
 *
 * @tparam Iterable Iterable container class. Iterable::iterator must meet
 * LegacyIterator requirements
 * @param it Object of type Iterable
 * @param cfg Progress configuration struct.
 *
 * @ingroup Support
 */
template <class Iterable>
inline auto ProgressWrap(Iterable&& it, ProgressConfig cfg)
{
    return ProgressBarIterable<Iterable>(
        std::forward<Iterable>(it), std::move(cfg));
}

/**
 * @copybrief
 *
 * @copydoc
 *
 * @tparam Iterable Iterable container class. Iterable::iterator must meet
 * LegacyIterator requirements
 * @param it Object of type Iterable.
 * @param label Label for generated progress bar. Overrides the label in the
 * provided cfg.
 * @param cfg Progress configuration struct.
 * @return
 */
template <class Iterable>
inline auto ProgressWrap(
    Iterable&& it, std::string label, ProgressConfig cfg = {})
{
    cfg.label = std::move(label);
    return ProgressWrap(std::forward<Iterable>(it), std::move(cfg));
}

/**
 * Create and connect a class which implements volcart::IterationsProgress to a
 * progress bar. This does not extend the lifetime of the passed object. The
 * returned ProgressBar formatting can be modified, except for the PostfixText,
 * which is overridden whenever the progress updates are received.
 *
 * @ingroup Support
 */
template <class ProgressEnabled>
inline auto ReportProgress(ProgressEnabled& p, ProgressConfig cfg = {})
{
    using namespace indicators;
    using namespace indicators::option;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    // Setup progress bar
    auto iters = p.progressIterations();
    auto progressBar = NewProgressBar(iters, cfg.label);
    // Set starting text color
    if (cfg.useColors) {
        progressBar->set_option(ForegroundColor{indicators::Color::yellow});
    }
    // Connect to progress updates
    auto startTime = std::make_shared<steady_clock::time_point>();
    p.progressUpdated.connect([progressBar, iters, startTime, cfg](auto p) {
        if (steady_clock::now() - *startTime < cfg.interval) {
            return;
        }
        *startTime = steady_clock::now();
        if (cfg.showIters) {
            auto post = std::to_string(p) + "/" + std::to_string(iters);
            progressBar->set_option(PostfixText{post});
        }
        progressBar->set_progress(p);
    });
    // Connect to progress completed
    p.progressComplete.connect([progressBar, iters, cfg]() {
        if (cfg.showIters) {
            auto post = std::to_string(iters) + "/" + std::to_string(iters);
            progressBar->set_option(PostfixText{post});
        }
        if (cfg.useColors) {
            progressBar->set_option(ForegroundColor{indicators::Color::green});
        }
        progressBar->set_progress(iters);
    });
    // Disconnect on completion
    p.progressComplete.connect([&p]() {
        p.progressStarted.disconnect();
        p.progressUpdated.disconnect();
        p.progressComplete.disconnect();
    });

    // Return the progress bar
    return progressBar;
}

/**
 * @copydoc ReportProgress(ProgressEnabled&, std::string, bool, bool)
 * @ingroup Support
 */
template <class ProgressEnabled>
inline auto ReportProgress(
    ProgressEnabled& p, const std::string& label, ProgressConfig cfg = {})
{
    cfg.label = label;
    return ReportProgress(p, cfg);
}

/**
 * @copydoc ReportProgress(ProgressEnabled&, std::string, bool, bool)
 * @ingroup Support
 */
template <class ProgressEnabled>
inline auto ReportProgress(
    ProgressEnabled& p, const char* label, ProgressConfig cfg = {})
{
    cfg.label = label;
    return ReportProgress(p, cfg);
}

/**
 * @copydoc ReportProgress(ProgressEnabled&, std::string, bool, bool)
 * @ingroup Support
 */
template <class ProgressEnabled>
inline auto ReportProgress(
    ProgressEnabled& p, bool showIters, ProgressConfig cfg = {})
{
    cfg.showIters = showIters;
    return ReportProgress(p, cfg);
}

}  // namespace volcart
