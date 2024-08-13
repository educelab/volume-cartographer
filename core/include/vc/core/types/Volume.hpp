#pragma once

/** @file */

#include <cstddef>
#include <cstdint>
#include <mutex>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/BoundingBox.hpp"
#include "vc/core/types/Cache.hpp"
#include "vc/core/types/DiskBasedObjectBaseClass.hpp"
#include "vc/core/types/LRUCache.hpp"
#include "vc/core/types/Reslice.hpp"

namespace volcart
{
/**
 * @class Volume
 * @author Sean Karlage
 *
 * @brief Volumetric image data
 *
 * Provides access to a volumetric dataset, such as a CT scan. By default,
 * slices are cached in memory using volcart::LRUCache.
 *
 * @ingroup Types
 */
// shared_from_this used in Python bindings
// https://en.cppreference.com/w/cpp/memory/enable_shared_from_this
class Volume : public DiskBasedObjectBaseClass,
               public std::enable_shared_from_this<Volume>
{
public:
    /** Bounding Box type */
    using Bounds = BoundingBox<double, 3>;

    /** Shared pointer type */
    using Pointer = std::shared_ptr<Volume>;

    /** Slice cache type */
    using SliceCache = Cache<int, cv::Mat>;

    /** Default slice cache type */
    using DefaultCache = LRUCache<int, cv::Mat, NoOpMutex>;

    /** Default slice cache capacity */
    static constexpr std::size_t DEFAULT_CAPACITY = 200;

    /**@{*/
    /** Default constructor. Cannot be constructed without path. */
    Volume() = delete;

    /** @brief Load the Volume from a directory path */
    explicit Volume(filesystem::path path);

    /** @brief Make a new Volume at the specified path */
    Volume(filesystem::path path, Identifier uuid, std::string name);

    /** @overload Volume(volcart::filesystem::path) */
    static auto New(filesystem::path path) -> Pointer;

    /** @overload Volume(volcart::filesystem::path, Identifier, std::string) */
    static auto New(filesystem::path path, Identifier uuid, std::string name)
        -> Pointer;
    /**@}*/

    /**@{*/
    /** @brief Get the slice width */
    auto sliceWidth() const -> int;
    /** @brief Get the slice height */
    auto sliceHeight() const -> int;
    /** @brief Get the number of slices */
    auto numSlices() const -> int;
    /** @brief Get the voxel size (in microns) */
    auto voxelSize() const -> double;
    /** @brief Get the minimum intensity value in the Volume */
    auto min() const -> double;
    /** @brief Get the maximum intensity value in the Volume */
    auto max() const -> double;
    /**@}*/

    /**@{*/
    /** @brief Set the expected width of the slice images */
    void setSliceWidth(int w);
    /** @brief Set the expected height of the slice images */
    void setSliceHeight(int h);
    /** @brief Set the expected number of slice images */
    void setNumberOfSlices(std::size_t numSlices);
    /** @brief Set the voxel size (in microns) */
    void setVoxelSize(double s);
    /** @brief Set the minimum value in the Volume */
    void setMin(double m);
    /** @brief Set the maximum value in the Volume */
    void setMax(double m);
    /**@}*/

    /**@{*/
    /** @brief Get the bounding box */
    auto bounds() const -> Bounds;
    /** @brief Return whether a position is within the volume bounds */
    auto isInBounds(double x, double y, double z) const -> bool;
    /** @overload isInBounds(double, double, double) const */
    auto isInBounds(const cv::Vec3d& v) const -> bool;
    /**@}*/

    /**@{*/
    /**
     * @brief Get a slice by index number
     *
     * @warning Because cv::Mat is essentially a pointer to a matrix, modifying
     * the slice returned by getSliceData() will modify the cached slice as
     * well. Use getSliceDataCopy() if the slice is to be modified.
     */
    auto getSliceData(int index) const -> cv::Mat;

    /** @copydoc getSliceData(int) const */
    auto getSliceDataCopy(int index) const -> cv::Mat;

    /** @brief Get slice by index and cut out a rect to return */
    auto getSliceDataRect(int index, cv::Rect rect) const -> cv::Mat;

    /** @brief Copy a slice by index and cut out a rect to return */
    auto getSliceDataRectCopy(int index, cv::Rect rect) const -> cv::Mat;

    /**
     * @brief Set a slice by index number
     *
     * Index must be less than the number of slices in the volume.
     *
     * @warning This will overwrite any existing slice data on disk.
     */
    void setSliceData(int index, const cv::Mat& slice, bool compress = true);

    /** @brief Get the file path of a slice by index */
    auto getSlicePath(int index) const -> filesystem::path;
    /**@}*/

    /**@{*/
    /** @brief Get the intensity value at a voxel position */
    auto intensityAt(int x, int y, int z) const -> std::uint16_t;

    /** @copydoc intensityAt() */
    auto intensityAt(const cv::Vec3d& v) const -> std::uint16_t;

    /**
     * @brief Get the intensity value at a subvoxel position
     *
     * Values are trilinearly interpolated.
     *
     * Trilinear interpolation equation from
     * <a href = "http://paulbourke.net/miscellaneous/interpolation/"> here</a>.
     */
    auto interpolateAt(double x, double y, double z) const -> std::uint16_t;

    /** @copydoc interpolateAt(double, double, double) const */
    auto interpolateAt(const cv::Vec3d& v) const -> std::uint16_t;

    /**
     * @brief Create a Reslice image by intersecting the volume with a plane
     *
     * @warning This function makes no attempt to check that the X and Y vectors
     * are orthogonal to each other. Vectors that are not orthogonal will
     * produce unexpected behavior.
     *
     * @param center Center of the Reslice image
     * @param xvec X-axis of the Reslice plane
     * @param yvec Y-axis of the Reslice plane
     * @param height Height of the Reslice image
     * @param width Width of the Reslice image
     */
    auto reslice(
        const cv::Vec3d& center,
        const cv::Vec3d& xvec,
        const cv::Vec3d& yvec,
        int width = 64,
        int height = 64) const -> Reslice;
    /**@}*/

    /**@{*/
    /** @brief Enable slice caching */
    void setCacheSlices(bool b);

    /** @brief Set the slice cache */
    void setCache(SliceCache::Pointer c) const;

    /** @brief Set the maximum number of cached slices */
    void setCacheCapacity(std::size_t newCacheCapacity) const;

    /** @brief Set the maximum size of the cache in bytes */
    void setCacheMemoryInBytes(std::size_t nbytes) const;

    /** @brief Get the maximum number of cached slices */
    auto getCacheCapacity() const -> std::size_t;

    /** @brief Get the current number of cached slices */
    auto getCacheSize() const -> std::size_t;

    /** @brief Purge the slice cache */
    void cachePurge() const;
    /**@}*/

protected:
    /** Slice width */
    int width_{0};
    /** Slice height */
    int height_{0};
    /** Number of slices */
    int slices_{0};
    /** Slice file name padding */
    int numSliceCharacters_{0};

    /** Whether to use slice cache */
    bool cacheSlices_{true};
    /** Slice cache */
    mutable SliceCache::Pointer cache_{DefaultCache::New(DEFAULT_CAPACITY)};
    /** Cache mutex for thread-safe access */
    mutable std::shared_mutex cacheMutex_;
    /** Per-slice mutexes */
    mutable std::vector<std::mutex> sliceMutexes_;

    /** Load slice from disk */
    cv::Mat load_slice_(int index) const;
    /** Load slice from cache */
    cv::Mat cache_slice_(int index) const;
};
}  // namespace volcart
