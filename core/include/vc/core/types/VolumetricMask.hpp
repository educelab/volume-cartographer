#pragma once

/** @file */

#include <memory>
#include <unordered_set>

#include "vc/core/types/PointSet.hpp"
#include "vc/core/util/HashFunctions.hpp"

namespace volcart
{

/**
 * @brief Stores per-voxel mask information for a volume
 *
 * @warning This container is built on std::unordered_set, so it's very fast
 * once built, but not very memory friendly.
 */
class VolumetricMask
{
public:
    /** Voxel type */
    using Voxel = cv::Vec3i;

private:
    /** Custom set type */
    using MaskSet = std::unordered_set<Voxel, Vec3iHash>;

public:
    /** Iterator type */
    using iterator = MaskSet::iterator;
    /** Const-iterator type */
    using const_iterator = MaskSet::const_iterator;

    /** Pointer type */
    using Pointer = std::shared_ptr<VolumetricMask>;

    /** Static New function for all constructors of T */
    template <typename... Args>
    static Pointer New(Args... args)
    {
        return std::make_shared<VolumetricMask>(std::forward<Args>(args)...);
    }

    /** @brief Default constructor */
    VolumetricMask() = default;

    /** @brief Constructor with existing Voxel container */
    template <class Container>
    explicit VolumetricMask(const Container& ps)
    {
        mask_.reserve(ps.size());
        mask_.insert(std::begin(ps), std::end(ps));
    }

    /** @brief Add Voxel to mask */
    void setIn(const Voxel& v);
    /** @brief Remove Voxel from mask */
    void setOut(const Voxel& v);

    /** @brief Add Voxels to mask */
    template <class Container>
    void setIn(const Container& ps)
    {
        mask_.reserve(mask_.size() + ps.size());
        mask_.insert(std::begin(ps), std::end(ps));
    }

    /** @brief Remove Voxels from the mask */
    template <class Container>
    void setOut(const Container& ps)
    {
        for (const auto& p : ps) {
            setOut(p);
        }
    }

    /** @brief Check whether a Voxel is in the mask */
    bool isIn(const Voxel& v) const;
    /** @brief Check whether a Voxel is not in the mask */
    bool isOut(const Voxel& v) const;

    /** @brief Check whether a sub-voxel is in the mask */
    bool isIn(const cv::Vec3d& v) const;
    /** @brief Check whether a sub-voxel is not in the mask */
    bool isOut(const cv::Vec3d& v) const;

    /** @brief Get a const-iterator to the first element in the mask */
    iterator begin() noexcept;
    /** @copydoc begin() */
    const_iterator begin() const noexcept;
    /** @copydoc begin() */
    const_iterator cbegin() const noexcept;

    /** @brief Get a const-iterator to one past the last element in the mask */
    iterator end() noexcept;
    /** @copydoc end() */
    const_iterator end() const noexcept;
    /** @copydoc end() */
    const_iterator cend() const noexcept;

    /** @brief Clear the mask of all voxels */
    void clear();

    /** @brief Get the list of masked points as a vector */
    std::vector<Voxel> as_vector() const;

private:
    /** Mask storage container */
    MaskSet mask_;
};

}  // namespace volcart
