#pragma once
#include <shared_mutex>
/** @file */

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
    using DefaultCache = LRUCache<int, cv::Mat>;

    /** Default slice cache capacity */
    static constexpr size_t DEFAULT_CAPACITY = 200;

    /**@{*/
    /** Default constructor. Cannot be constructed without path. */
    Volume() = delete;

    /** @brief Load the Volume from a directory path */
    explicit Volume(volcart::filesystem::path path);

    /** @brief Make a new Volume at the specified path */
    Volume(volcart::filesystem::path path, Identifier uuid, std::string name);

    /** @overload Volume(volcart::filesystem::path) */
    static Pointer New(volcart::filesystem::path path);

    /** @overload Volume(volcart::filesystem::path, Identifier, std::string) */
    static Pointer New(
        volcart::filesystem::path path, Identifier uuid, std::string name);
    /**@}*/

    /**@{*/
    /** @brief Get the slice width */
    int sliceWidth() const;
    /** @brief Get the slice height */
    int sliceHeight() const;
    /** @brief Get the number of slices */
    int numSlices() const;
    /** @brief Get the voxel size (in microns) */
    double voxelSize() const;
    /** @brief Get the minimum intensity value in the Volume */
    double min() const;
    /** @brief Get the maximum intensity value in the Volume */
    double max() const;
    /**@}*/

    /**@{*/
    /** @brief Set the expected width of the slice images */
    void setSliceWidth(int w);
    /** @brief Set the expected height of the slice images */
    void setSliceHeight(int h);
    /** @brief Set the expected number of slice images */
    void setNumberOfSlices(size_t numSlices);
    /** @brief Set the voxel size (in microns) */
    void setVoxelSize(double s);
    /** @brief Set the minimum value in the Volume */
    void setMin(double m);
    /** @brief Set the maximum value in the Volume */
    void setMax(double m);
    /**@}*/

    /**@{*/
    /** @brief Get the bounding box */
    Bounds bounds() const;
    /** @brief Return whether a position is within the volume bounds */
    bool isInBounds(double x, double y, double z) const;
    /** @overload isInBounds(double, double, double) const */
    bool isInBounds(const cv::Vec3d& v) const;
    /**@}*/

    /**@{*/
    /**
     * @brief Get a slice by index number
     *
     * @warning Because cv::Mat is essentially a pointer to a matrix, modifying
     * the slice returned by getSliceData() will modify the cached slice as
     * well. Use getSliceDataCopy() if the slice is to be modified.
     */
    cv::Mat getSliceData(int index) const;

    /** @copydoc getSliceData(int) const */
    cv::Mat getSliceDataCopy(int index) const;

    /** @brief Get slice by index and cut out a rect to return */
    cv::Mat getSliceDataRect(int index, cv::Rect rect) const;

    /** @brief Copy a slice by index and cut out a rect to return */
    cv::Mat getSliceDataRectCopy(int index, cv::Rect rect) const;

    /**
     * @brief Set a slice by index number
     *
     * Index must be less than the number of slices in the volume.
     *
     * @warning This will overwrite any existing slice data on disk.
     */
    void setSliceData(int index, const cv::Mat& slice, bool compress = true);

    /** @brief Get the file path of a slice by index */
    volcart::filesystem::path getSlicePath(int index) const;
    /**@}*/

    /**@{*/
    /** @brief Get the intensity value at a voxel position */
    uint16_t intensityAt(int x, int y, int z) const;

    /** @copydoc intensityAt() */
    uint16_t intensityAt(const cv::Vec3d& v) const
    {
        return intensityAt(int(v[0]), int(v[1]), int(v[2]));
    }

    /**
     * @brief Get the intensity value at a subvoxel position
     *
     * Values are trilinearly interpolated.
     *
     * Trilinear interpolation equation from
     * <a href = "http://paulbourke.net/miscellaneous/interpolation/"> here</a>.
     */
    uint16_t interpolateAt(double x, double y, double z) const;

    /** @copydoc interpolateAt(double, double, double) const */
    uint16_t interpolateAt(const cv::Vec3d& v) const
    {
        return interpolateAt(v[0], v[1], v[2]);
    }

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
    Reslice reslice(
        const cv::Vec3d& center,
        const cv::Vec3d& xvec,
        const cv::Vec3d& yvec,
        int width = 64,
        int height = 64) const;
    /**@}*/

    /**@{*/
    /** @brief Enable slice caching */
    void setCacheSlices(bool b) { cacheSlices_ = b; }

    /** @brief Set the slice cache */
    void setCache(SliceCache::Pointer c) { cache_ = std::move(c); }

    /** @brief Set the maximum number of cached slices */
    void setCacheCapacity(size_t newCacheCapacity)
    {
        cache_->setCapacity(newCacheCapacity);
    }

    /** @brief Set the maximum size of the cache in bytes */
    void setCacheMemoryInBytes(size_t nbytes)
    {
        // x2 because pixels are 16 bits normally. Not a great solution.
        setCacheCapacity(nbytes / (sliceWidth() * sliceHeight() * 2));
    }

    /** @brief Get the maximum number of cached slices */
    size_t getCacheCapacity() const { return cache_->capacity(); }

    /** @brief Get the current number of cached slices */
    size_t getCacheSize() const { return cache_->size(); }

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
    mutable std::mutex cacheMutex_;

    /** Load slice from disk */
    cv::Mat load_slice_(int index) const;
    /** Load slice from cache */
    cv::Mat cache_slice_(int index) const;
    /** Shared mutex for thread-safe access */
    mutable std::shared_mutex cache_mutex_;
};
}  // namespace volcart
