#pragma once

#include <functional>
#include <type_traits>
#include <vector>

namespace volcart
{

/**
 * @brief Basic signal class for implementing event callbacks
 *
 * This class provides a statically-typed signals implementation similar to what
 * you would find in Qt or Boost.signals2. Parameters passed to send() are
 * immediately passed by value to connected functions. There is currently no
 * lifetime management for connected objects, so this class is not thread-safe
 * and should only be used to implement simple callbacks.
 *
 * Example Usage:
 * @code{.cpp}
 * void IntFn(int i) {
 *     std::cout << i << std::endl;
 * }
 *
 * Signal<int> signal;
 * signal.connect(IntFn);
 * signal.send(1); // prints "1"
 * @endcode
 *
 * @tparam Types List of types to be emitted by the signal
 *
 * @ingroup Util
 */
template <class... Types>
class Signal
{
public:
    /** Type of functions to which this signal can be connected */
    using SlotFnType = std::function<void(Types...)>;

    /** @brief Connect to a free, static, or lambda function */
    void connect(SlotFnType slot)
    {
        connections_.emplace_back(std::move(slot));
    }

    /**
     * @brief Connect to an object's member function
     *
     * @param obj Pointer to an object
     * @param fn Reference to a member function of obj
     */
    template <class Obj, class ObjMemberFn>
    void connect(Obj* obj, ObjMemberFn&& fn)
    {
        connections_.emplace_back(
            [=](Types... args) { return (*obj.*fn)(args...); });
    }

    /**
     * @brief Connect a valued signal to a no-parameter object member function
     */
    template <class Obj, class Ret>
    void connect(Obj* obj, Ret (Obj::*fn)())
    {
        connections_.emplace_back(
            [=](Types... /*unused*/) { return (*obj.*fn)(); });
    }

    /**
     * @brief Connect a valued signal to a no-parameter object member function
     *
     * cv-qualified version.
     */
    template <class Obj, class Ret>
    void connect(const Obj* obj, Ret (Obj::*fn)() const)
    {
        connections_.emplace_back(
            [=](Types... /*unused*/) { return (*obj.*fn)(); });
    }

    /**
     * @brief Connect signals with parameters to functions without parameters
     */
    template <
        class Enabled = typename std::enable_if<
            std::is_empty<std::tuple<Types...>(Types...)>::value,
            bool>>
    void connect(const std::function<void()>& slot)
    {
        connections_.emplace_back([=](Types&&... /*unused*/) { slot(); });
    }

    /** @brief Remove all connections */
    void disconnect() { connections_.clear(); }

    /** @brief Get the number of connected slots */
    size_t numConnections() const { return connections_.size(); }

    /** @brief Signal all connections with parameters */
    void send(Types... args)
    {
        for (const auto& c : connections_) {
            c.slot(args...);
        }
    }

    /** @brief Convenience operator: Calls Signal::send(args) */
    void operator()(Types... args) { send(std::forward<Types>(args)...); }

private:
    /** Basic connection struct */
    struct Connection {
        Connection(const SlotFnType& f) { slot = f; }
        SlotFnType slot;
    };
    /** List of connections */
    std::vector<Connection> connections_;
};

/** Specialization of connect for signals without parameters */
template <>
template <class Obj, class ObjMemberFn>
void Signal<>::connect(Obj* obj, ObjMemberFn&& fn)
{
    connections_.emplace_back([=]() { return (*obj.*fn)(); });
}

/** Full specialization of operator() for signals without parameters */
template <>
inline void Signal<>::operator()()
{
    send();
}

}  // namespace volcart