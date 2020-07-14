#pragma once

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
        opt::MaxProgress{static_cast<int>(maxProg)},
        opt::ShowPercentage{true},
        opt::ShowElapsedTime{true},
        opt::ShowRemainingTime{true},
        opt::Start{" ["},
        opt::Fill{"■"},
        opt::Lead{"■"},
        opt::Remainder{" "},
        opt::End{"]"},
        opt::ForegroundColor{Color::white},
        opt::FontStyles{std::vector<FontStyle>{FontStyle::bold}
        });
    // clang-format on
}

/**
 * Iterable wrapper for tracking iteration progress. This class's iterator meets
 * the LegacyIterator requirements. Each call to `begin()` initializes a new
 * indicators::ProgressBar on the heap. Incrementing the returned iterator
 * advances the progress bar by one iteration. Copies of the iterator will
 * share the same progress bar instance.
 *
 * volcart::ProgressWrap provides a convenient method for constructing a
 * ProgressBarIterable for range-based for loops.
 *
 * @code
 *
 * using Vector = std::vector<int>;
 * using Iterator = Vector::iterator;
 * using ProgressBar = ProgressBarIterable<Iterator>;
 *
 * // Using ProgressBarIterable
 * Vector integers{0, 1, 2};
 * ProgressBar prog(integers.begin(), integers.end(), "Integers");
 * for(auto& i : prog) {
 *     i += 1;
 * }
 * // integers == {1, 2, 3}
 *
 * // Using ConsoleProgressBar
 * for(auto& i : ProgressWrap(integers)) {
 *     i += 1;
 * }
 * // integers == {2, 3, 4}
 *
 * @endcode
 *
 * @ingroup Support
 */
template <class IteratorType>
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
            T beginIt, size_t max, std::string label)
            : it_{beginIt}, maxProg_{max}
        {
            using indicators::Color;
            using indicators::option::ForegroundColor;
            bar_ = NewProgressBar(maxProg_, std::move(label));
            bar_->set_option(ForegroundColor{Color::yellow});
        }

        /** Constructor for the end() iterator */
        explicit ConsoleProgressIterator(T endIt) : it_{endIt} {}

        /** Destructor */
        ~ConsoleProgressIterator() { indicators::show_console_cursor(true); }

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
                indicators::show_console_cursor(false);
                bar_->set_progress(prog_);
            } else {
                bar_->set_option(ForegroundColor{Color::green});
                bar_->tick();
                indicators::show_console_cursor(true);
            }
            return *this;
        }
    };

public:
    using iterator = ConsoleProgressIterator<IteratorType>;
    using const_iterator = ConsoleProgressIterator<const IteratorType>;

    /** Constructor for range with label (optional) */
    explicit ProgressBarIterable(
        IteratorType begin, IteratorType end, std::string label = "")
        : begin_{std::move(begin)}
        , end_{std::move(end)}
        , label_{std::move(label)}
    {
    }

    /** Destructor */
    ~ProgressBarIterable() { indicators::show_console_cursor(true); }

    /** Get a new ProgressIterable::iterator. Instantiates a new progress bar.
     */
    iterator begin()
    {
        return iterator{begin_, std::distance(begin_, end_), label_};
    }

    /**
     * Get the end-valued ProgressIterable::iterator. Does not instantiate a
     * new progress bar.
     */
    iterator end() { return iterator{end_}; }

    /**
     * Get a new ProgressIterable::const_iterator. Instantiates a new progress
     * bar.
     */
    const_iterator cbegin() const
    {
        return const_iterator{begin_, std::distance(begin_, end_), label_};
    }

    /**
     * Get the end-valued ProgressIterable::const_iterator. Does not instantiate
     * a new progress bar.
     */
    const_iterator cend() const { return const_iterator{end_}; }

private:
    /** Copy of range begin iterator */
    IteratorType begin_;
    /** Copy of range end iterator */
    IteratorType end_;
    /** Progress bar label */
    std::string label_;
};

/**
 * @brief Wrap a progress bar around an Iterable object.
 *
 * Returns a ProgressBarIterable which wraps the provided Iterable object.
 *
 * @tparam Iterable Iterable container class. Iterable::iterator must meet
 * LegacyIterator requirements
 * @param it Object of type Iterable
 * @param label Label for generated progress bar (optional)
 *
 * @ingroup Support
 */
template <class Iterable>
inline auto ProgressWrap(Iterable&& it, std::string label = "")
{
    using IteratorType = decltype(std::begin(it));
    return ProgressBarIterable<IteratorType>(
        std::begin(it), std::end(it), std::move(label));
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
    ProgressEnabled& p, std::string label, bool showIters)
{
    using namespace indicators;
    using namespace indicators::option;
    // Setup progress bar
    auto iters = p.progressIterations();
    auto progressBar = NewProgressBar(iters, std::move(label));
    // Set starting text color
    progressBar->set_option(ForegroundColor{Color::yellow});
    // Connect to progress started
    p.progressStarted.connect([]() { show_console_cursor(false); });
    // Connect to progress updates
    p.progressUpdated.connect([progressBar, iters, showIters](auto p) {
        if (showIters) {
            auto post = std::to_string(p) + "/" + std::to_string(iters);
            progressBar->set_option(PostfixText{post});
        }
        progressBar->set_progress(p);
    });
    // Connect to progress completed
    p.progressComplete.connect([progressBar, iters, showIters]() {
        if (showIters) {
            auto post = std::to_string(iters) + "/" + std::to_string(iters);
            progressBar->set_option(PostfixText{post});
        }
        progressBar->set_option(ForegroundColor{Color::green});
        progressBar->tick();
        show_console_cursor(true);
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
 * @copydoc ReportProgress(ProgressEnabled&, std::string, bool)
 * @ingroup Support
 */
template <class ProgressEnabled>
inline auto ReportProgress(ProgressEnabled& p)
{
    return ReportProgress(p, "", true);
}

/**
 * @copydoc ReportProgress(ProgressEnabled&, std::string, bool)
 * @ingroup Support
 */
template <class ProgressEnabled>
inline auto ReportProgress(ProgressEnabled& p, std::string label)
{
    return ReportProgress(p, std::move(label), true);
}

/**
 * @copydoc ReportProgress(ProgressEnabled&, std::string, bool)
 * @ingroup Support
 */
template <class ProgressEnabled>
inline auto ReportProgress(ProgressEnabled& p, const char* label)
{
    return ReportProgress(p, label, true);
}

/**
 * @copydoc ReportProgress(ProgressEnabled&, std::string, bool)
 * @ingroup Support
 */
template <class ProgressEnabled>
inline auto ReportProgress(ProgressEnabled& p, bool showIters)
{
    return ReportProgress(p, "", showIters);
}

}  // namespace volcart
