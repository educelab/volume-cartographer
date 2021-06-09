#pragma once

/** @file */

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

template <class Iterable>
inline auto ProgressWrap(
    Iterable&& it, std::string label = "", bool useColors = false);

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
            T beginIt, size_t max, std::string label, bool useColors)
            : it_{beginIt}, maxProg_{max}, useColors_{useColors}
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
        reference operator*() const { return *it_; }

        /** Equality comparison: defer to wrapped iterators */
        bool operator==(const ConsoleProgressIterator& other) const
        {
            return it_ == other.it_;
        }

        /** Inequality comparison */
        bool operator!=(const ConsoleProgressIterator& other) const
        {
            return !(*this == other);
        }

        /** Increment the wrapped iterator and progress bar */
        ConsoleProgressIterator& operator++()
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
                if (useColors_) {
                    indicators::show_console_cursor(false);
                }
                bar_->set_progress(prog_);
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
    ProgressBarIterable(Iterable&& container, std::string label, bool useColors)
        : container_{std::forward<Iterable>(container)}
        , label_{std::move(label)}
        , useColors_{useColors}
    {
    }

    friend auto ProgressWrap<Iterable>(
        Iterable&& it, std::string label, bool useColors);

public:
    using iterator = ConsoleProgressIterator<IteratorType>;
    using const_iterator = ConsoleProgressIterator<const IteratorType>;

    /** Destructor */
    ~ProgressBarIterable()
    {
        if (useColors_) {
            indicators::show_console_cursor(true);
        }
    }

    /** Get a new ProgressIterable::iterator. Instantiates a new progress bar.
     */
    iterator begin()
    {
        auto begin = std::begin(container_);
        auto end = std::end(container_);
        return iterator{
            begin, static_cast<size_t>(std::distance(begin, end)), label_,
            useColors_};
    }

    /**
     * Get the end-valued ProgressIterable::iterator. Does not instantiate a
     * new progress bar.
     */
    iterator end() { return iterator{std::end(container_)}; }

    /**
     * Get a new ProgressIterable::const_iterator. Instantiates a new progress
     * bar.
     */
    const_iterator cbegin() const
    {
        auto begin = std::begin(container_);
        auto end = std::end(container_);
        return const_iterator{
            begin, std::distance(begin, end), label_, useColors_};
    }

    /**
     * Get the end-valued ProgressIterable::const_iterator. Does not instantiate
     * a new progress bar.
     */
    const_iterator cend() const { return const_iterator{std::end(container_)}; }

private:
    Iterable container_;
    /** Progress bar label */
    std::string label_;
    /** Use colors */
    bool useColors_{false};
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
 * @param label Label for generated progress bar (optional)
 *
 * @ingroup Support
 */
template <class Iterable>
inline auto ProgressWrap(Iterable&& it, std::string label, bool useColors)
{
    return ProgressBarIterable<Iterable>(
        std::forward<Iterable>(it), std::move(label), useColors);
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
inline auto ReportProgress(
    ProgressEnabled& p, std::string label, bool showIters, bool useColors)
{
    using namespace indicators;
    using namespace indicators::option;
    // Setup progress bar
    auto iters = p.progressIterations();
    auto progressBar = NewProgressBar(iters, std::move(label));
    // Set starting text color
    if (useColors) {
        progressBar->set_option(ForegroundColor{Color::yellow});
    }
    // Connect to progress updates
    p.progressUpdated.connect([progressBar, iters, showIters](auto p) {
        if (showIters) {
            auto post = std::to_string(p) + "/" + std::to_string(iters);
            progressBar->set_option(PostfixText{post});
        }
        progressBar->set_progress(p);
    });
    // Connect to progress completed
    p.progressComplete.connect([progressBar, iters, showIters, useColors]() {
        if (showIters) {
            auto post = std::to_string(iters) + "/" + std::to_string(iters);
            progressBar->set_option(PostfixText{post});
        }
        if (useColors) {
            progressBar->set_option(ForegroundColor{Color::green});
        }
        progressBar->tick();
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
inline auto ReportProgress(ProgressEnabled& p)
{
    return ReportProgress(p, "", true, false);
}

/**
 * @copydoc ReportProgress(ProgressEnabled&, std::string, bool, bool)
 * @ingroup Support
 */
template <class ProgressEnabled>
inline auto ReportProgress(ProgressEnabled& p, std::string label)
{
    return ReportProgress(p, std::move(label), true, false);
}

/**
 * @copydoc ReportProgress(ProgressEnabled&, std::string, bool, bool)
 * @ingroup Support
 */
template <class ProgressEnabled>
inline auto ReportProgress(ProgressEnabled& p, const char* label)
{
    return ReportProgress(p, label, true, false);
}

/**
 * @copydoc ReportProgress(ProgressEnabled&, std::string, bool, bool)
 * @ingroup Support
 */
template <class ProgressEnabled>
inline auto ReportProgress(ProgressEnabled& p, bool showIters)
{
    return ReportProgress(p, "", showIters, false);
}

}  // namespace volcart
