#pragma once

/** @file */

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
    ParticleChain& operator+=(const ForceChain& rhs);

    /** @brief Multiply each element of chain by a constant scale factor */
    ParticleChain& operator*=(const double& rhs);

    /** @brief Element access operator */
    auto operator[](size_t i) { return data_[i]; }

    /** @copydoc operator[](size_t) */
    auto operator[](size_t i) const { return data_[i]; }

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
    size_t size() { return data_.size(); }

    /** @copydoc size() */
    size_t size() const { return data_.size(); }

    /** @brief Empties and resets the chain */
    void clear() { data_.clear(); }

private:
    /** Data storage vector */
    Chain data_;
};

/** Free function operator for ParticleChain and ForceChain addition */
ParticleChain operator+(ParticleChain lhs, const ForceChain& rhs);
/** @copydoc operator+(ParticleChain, const ForceChain&) */
ParticleChain operator+(const ForceChain& rhs, ParticleChain lhs);
/** Free function operator for multiplying a ParticleChain by a scalar */
ParticleChain operator*(ParticleChain lhs, const double& rhs);
/** @copydoc operator*(ParticleChain, const double&) */
ParticleChain operator*(const double& rhs, ParticleChain lhs);
}  // namespace volcart::segmentation
