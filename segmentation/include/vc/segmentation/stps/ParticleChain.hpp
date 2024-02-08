#pragma once

/** @file */

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

#include "vc/segmentation/stps/ForceChain.hpp"
#include "vc/segmentation/stps/Particle.hpp"

namespace volcart::segmentation
{
/**
 * @class ParticleChain
 * @brief A simple class for keeping track of a connected chain of Particle
 * objects
 *
 * @ingroup stps
 */
class ParticleChain
{
public:
    /** Data storage type */
    using Chain = std::vector<Particle>;

    /** @brief Default constructor */
    ParticleChain() = default;

    /** @brief Constructor with chain initialization */
    explicit ParticleChain(Chain c) : data_{std::move(c)} {}

    /**
     * @brief Add a list of offset vectors to each element in the chain
     *
     * Offset vectors are added per element: `PC[0] + FC[0]; PC[1] + FC[1]; ...`
     * Throws `std::domain_error` if ForceChain size doesn't match the size of
     * the ParticleChain.
     */
    auto operator+=(const ForceChain& rhs) -> ParticleChain&;

    /** @brief Multiply each element of chain by a constant scale factor */
    auto operator*=(const double& rhs) -> ParticleChain&;

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
    void push_back(const Particle& val) { data_.push_back(val); }

    /** @brief Returns the number of elements in the chain */
    auto size() -> std::size_t { return data_.size(); }

    /** @copydoc size() */
    auto size() const -> std::size_t { return data_.size(); }

    /** @brief Empties and resets the chain */
    void clear() { data_.clear(); }

private:
    /** Data storage vector */
    Chain data_;
};

/** Free function operator for ParticleChain and ForceChain addition */
auto operator+(ParticleChain lhs, const ForceChain& rhs) -> ParticleChain;
/** @copydoc operator+(ParticleChain, const ForceChain&) */
auto operator+(const ForceChain& rhs, ParticleChain lhs) -> ParticleChain;
/** Free function operator for multiplying a ParticleChain by a scalar */
auto operator*(ParticleChain lhs, const double& rhs) -> ParticleChain;
/** @copydoc operator*(ParticleChain, const double&) */
auto operator*(const double& rhs, ParticleChain lhs) -> ParticleChain;
}  // namespace volcart::segmentation
