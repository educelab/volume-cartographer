#pragma once

/** @file */

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

namespace volcart::segmentation
{
/** Force type */
using Force = cv::Vec3d;
/**
 * @class ForceChain
 * @brief A simple class for tracking a list of offset vectors ("forces")
 *
 * Vectors in this class are assumed to be 3D offsets to the position of
 * elements in a ParticleChain.
 *
 * @ingroup stps
 */
class ForceChain
{
public:
    /** Data storage type */
    using Chain = std::vector<Force>;

    /** @brief Default constructor */
    ForceChain() = default;

    /** @brief Constructor with chain initialization */
    explicit ForceChain(Chain c) : data_{std::move(c)} {}

    /**
     * @brief Add a list of offset vectors to each element in the chain
     *
     * Offset vectors are added per element: `C1[0] + C2[0]; C1[1] + C2[1]; ...`
     * Throws `std::domain_error` if ForceChain size doesn't match the size of
     * this ForceChain.
     */
    auto operator+=(const ForceChain& rhs) -> ForceChain&;

    /** @brief Multiply each element of chain by a constant scale factor */
    auto operator*=(const double& rhs) -> ForceChain&;

    /** @brief Element access operator */
    auto operator[](std::size_t i) { return data_[i]; }

    /** @copydoc operator[](std::size_t) */
    auto operator[](std::size_t i) const { return data_[i]; }

    /** @brief Returns an iterator to the beginning of the chain */
    auto begin() { return data_.begin(); }

    /** @copydoc begin() */
    auto begin() const { return data_.begin(); }

    /** @brief Returns an iterator to the end of the chain */
    auto end() { return data_.end(); }

    /** @copydoc end() */
    auto end() const { return data_.end(); }

    /** @brief Constructs an element at the end of the chain */
    template <class... Args>
    void emplace_back(Args&&... args)
    {
        data_.emplace_back(std::forward<Args>(args)...);
    }

    /** @brief Adds an element to the end of the chain */
    void push_back(const Force& val) { data_.push_back(val); }

    /** @brief Returns the number of elements in the chain */
    auto size() -> std::size_t { return data_.size(); }

    /** @copydoc size() */
    auto size() const -> std::size_t { return data_.size(); }

    /** @brief Empties and resets the chain */
    void clear() { data_.clear(); }

    /**
     * @brief Normalize the magnitude of each Force in the chain
     *
     * @param alpha Upper value to which the magnitude is normalized
     */
    static void Normalize(ForceChain& c, double alpha = 1.0);

private:
    /** Data storage vector */
    Chain data_;
};

/** Free function operator for per-element ForceChain addition */
auto operator+(ForceChain lhs, const ForceChain& rhs) -> ForceChain;
/** Free function operator for multiplying a ForceChain by a scalar */
auto operator*(ForceChain lhs, const double& rhs) -> ForceChain;
/** @copydoc operator*(ForceChain, const double&) */
auto operator*(const double& rhs, ForceChain lhs) -> ForceChain;
}  // namespace volcart::segmentation
