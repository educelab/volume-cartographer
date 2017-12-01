#pragma once

#include <array>
#include <string>

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>

#include "vc/core/types/BoundingBox.hpp"
#include "vc/core/types/LRUCache.hpp"
#include "vc/core/types/Metadata.hpp"
#include "vc/core/types/Reslice.hpp"
#include "vc/core/types/Tensor3D.hpp"

namespace volcart
{
/** Voxel type */
using Voxel = cv::Vec3d;

/** Eigenvalue type */
using EigenValue = double;

/** Eigenvector type */
using EigenVector = cv::Vec3d;

/** EigenPair type */
using EigenPairs = std::array<std::pair<EigenValue, EigenVector>, 3>;

/** StructureTensor type */
using StructureTensor = cv::Matx33d;

/** Zero-valued StructureTensor */
static const auto ZERO_STRUCTURE_TENSOR =
    StructureTensor(0, 0, 0, 0, 0, 0, 0, 0, 0);

using Neighborhood = std::vector<uint16_t>;

/** @brief Neighborhood Directional Filtering Options
 *
 * Bidirectional: Consider data in both the positive and negative normal
 * direction \n
 * Positive: Only consider data in the positive normal direction \n
 * Negative: Only consider data in the negative normal direction \n
 */
enum class Direction { Bidirectional, Positive, Negative };

/**
 * @class Volume
 * @author Sean Karlage
 *
 * @brief Volumetric image data
 *
 * Provides access to a volumetric dataset, such as a CT scan. Since these
 * datasets are often very large, this class is streaming in nature: the volume
 * slices are stored on disk and cached in memory when needed.
 *
 * @ingroup Types
 */
class Volume
{
public:
    /** @enum Axis labels */
    enum class Axis { X, Y };

    /** Shared pointer type */
    using Pointer = std::shared_ptr<Volume>;

    /** Volume ID type */
    using Identifier = std::string;

    /** ID/Name paid */
    using Description = std::pair<Identifier, std::string>;

    /** Bounding Box type */
    using Bounds = BoundingBox<double, 3>;

    /**@{*/
    /**
     * Default constructor. The Volume class is always backed by on-disk
     * information. If you want to use your slice files as a Volume, you have
     * two options:
     *  - Volume(boost::filesystem::path path) if the directory already has a
     *  metadata file.
     *  - Volume(boost::filesystem::path, std::string, std::string) if the
     *  directory doesn't already have a metadata file.
     */
    Volume() = delete;

    /**
     * @brief Load a Volume from file
     *
     * The path pointed to by `path` is expected to provide a JSON metadata file
     * that describes the Volume. Use
     * Volume(boost::filesystem::path, std::string, std::string) if the
     * directory does not have such a file.
     */
    explicit Volume(boost::filesystem::path path);

    /**
     * @brief Load a directory of slices as a Volume
     *
     * Should be used when the directory being loaded does not already have
     * a metadata file. It's a good idea to set the slice dimensions, number of
     * slices, and the voxel size of the Volume at a minimum. After being set,
     * the metadata can be written to disk using saveMetadata() and the Volume
     * can be reloaded in the future using Volume(boost::filesystem::path).
     *
     * @param path Path to the Volume root directory
     * @param uuid "Unique" ID of the Volume
     * @param name Human-readable name for the Volume
     */
    Volume(boost::filesystem::path path, std::string uuid, std::string name);

    /** @brief Construct a Volume using existing slice data
     *
     * Slice files in slicePath must be zero-padded, indexed from zero, and have
     * the file extension .tif.
     *
     * @param slicePath Path to slice data
     * @param nslices Number of slices in the Volume
     * @param sliceHeight Slice height (in pixels)
     * @param sliceWidth Slice width (in pixels)
     */
    Volume(
        boost::filesystem::path slicePath,
        int32_t nslices,
        int32_t sliceWidth,
        int32_t sliceHeight)
        : path_(std::move(slicePath))
        , numSlices_(nslices)
        , sliceWidth_(sliceWidth)
        , sliceHeight_(sliceHeight)
    {
        numSliceCharacters_ = std::to_string(nslices).size();
    }

    /** @copydoc Volume(boost::filesystem::path) */
    static Pointer New(boost::filesystem::path path);

    /** @copydoc Volume(boost::filesystem::path, std::string, std::string) */
    static Pointer New(
        boost::filesystem::path path, Identifier uuid, std::string name);
    /**@}*/

    /**@{*/
    /** @brief Get the "unique" Volume ID */
    Identifier id() { return metadata_.get<std::string>("uuid"); }

    /** @brief Get the human-readable name for the Volume */
    std::string name() { return metadata_.get<std::string>("name"); }

    /** @brief Get the slice width */
    int32_t sliceWidth() const { return metadata_.get<int>("width"); }

    /** @brief Get the slice height */
    int32_t sliceHeight() const { return sliceHeight_; }

    /** @brief Get the number of slices */
    int32_t numSlices() const { return numSlices_; }

    /** @brief Get the bounding box */
    Bounds bounds() const
    {
        return {{0, 0, 0}, {sliceWidth_, sliceHeight_, numSlices_}};
    }

    /** @brief Get the voxel size (in microns) */
    double voxelSize() const { return metadata_.get<double>("voxelsize"); }

    /** @brief Return whether a position is within the volume bounds */
    bool isInBounds(const cv::Vec3d& v) const
    {
        return v(0) >= 0 && v(0) < sliceWidth_ && v(1) >= 0 &&
               v(1) < sliceHeight_ && v(2) >= 0 && v(2) < numSlices_;
    }

    /** @brief Set the human-readable name of the volume */
    void setName(std::string n) { metadata_.set("name", std::move(n)); }

    /** @brief Set the expected width of the slice images */
    void setSliceWidth(int32_t w)
    {
        sliceWidth_ = w;
        metadata_.set("width", w);
    }

    /** @brief Set the expected height of the slice images */
    void setSliceHeight(int32_t h)
    {
        sliceHeight_ = h;
        metadata_.set("height", h);
    }

    /** @brief Set the voxel size (in microns) */
    void setVoxelSize(double s) { metadata_.set("voxelsize", s); }

    /** @brief Set the minimum value in the Volume */
    void setMin(double m) { metadata_.set("min", m); }

    /** @brief Set the maximum value in the Volume */
    void setMax(double m) { metadata_.set("max", m); }

    /** @brief Set the expected number of slice images */
    void setNumberOfSlices(size_t numSlices)
    {
        numSlices_ = numSlices;
        numSliceCharacters_ = std::to_string(numSlices_).size();
        metadata_.set("slices", numSlices);
    }

    /** @brief Update metadata on disk */
    void saveMetadata() { metadata_.save(); }
    /**@}*/

    /**@{*/
    /**
     * @brief Get a slice by index number
     *
     * Returning const ref doesn't actually forbid modifying cv::Mat data by the
     * compiler, but it should serve as a warning to the programmer that you
     * shouldn't monkey around with it.
     */
    const cv::Mat& getSliceData(int32_t index) const;

    /** @brief Get a unique copy of a slice by index number */
    cv::Mat getSliceDataCopy(int32_t index) const;

    /**
     * @brief Set a slice by index number
     *
     * Index must be less than the number of slices in the volume.
     *
     * @warning This will overwrite any existing slice data on disk.
     */
    void setSliceData(int32_t index, const cv::Mat& slice);

    /** @brief Get the file path of a slice by index */
    boost::filesystem::path getSlicePath(int32_t index) const;
    /**@}*/

    /**@{*/
    /** @brief Set the maximum number of cached slices */
    void setCacheCapacity(size_t newCacheCapacity)
    {
        cache_.setCapacity(newCacheCapacity);
    }

    /** @brief Set the maximum size of the cache in bytes */
    void setCacheMemoryInBytes(size_t nbytes)
    {
        // x2 because pixels are 16 bits normally. Not a great solution.
        setCacheCapacity(nbytes / (sliceWidth_ * sliceHeight_ * 2));
    }

    /** @brief Get the maximum number of cached slices */
    size_t getCacheCapacity() const { return cache_.capacity(); }

    /** @brief Get the current number of cached slices */
    size_t getCacheSize() const { return cache_.size(); }
    /**@}*/

    /**@{*/
    /**
     * @brief Get the intensity value at a subvoxel position
     *
     * Values are trilinearly interpolated.
     *
     * Trilinear interpolation equation from
     * <a href = "http://paulbourke.net/miscellaneous/interpolation/"> here</a>.
     */
    uint16_t interpolateAt(const Voxel& point) const;

    /** @copydoc interpolateAt() */
    uint16_t interpolatedIntensityAt(const Voxel& nonGridPoint) const
    {
        return interpolateAt(nonGridPoint);
    }

    /** @copydoc interpolatedIntensityAt() */
    uint16_t interpolatedIntensityAt(double x, double y, double z) const
    {
        return interpolateAt({x, y, z});
    }

    /** @brief Get the intensity value at a voxel position */
    uint16_t intensityAt(int32_t x, int32_t y, int32_t z) const
    {
        // clang-format off
        if (x < 0 || x >= sliceWidth_ ||
            y < 0 || y >= sliceHeight_ ||
            z < 0 || z >= numSlices_) {
            return 0;
        }
        // clang-format on
        return getSliceData(z).at<uint16_t>(y, x);
    }

    /** @copydoc intensityAt() */
    uint16_t intensityAt(const cv::Vec3d& v) const
    {
        return intensityAt(int32_t(v(0)), int32_t(v(1)), int32_t(v(2)));
    }
    /**@}*/

    /**@{*/
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
        const Voxel& center,
        const cv::Vec3d& xvec,
        const cv::Vec3d& yvec,
        int32_t width = 64,
        int32_t height = 64) const;

    /**
     * @brief Compute the structure tensor for a voxel position
     *
     * The structure tensor is calculated from the gradient of a cubic subvolume
     * centered around the provided point. The size of this subvolume is defined
     * by `voxelRadius`.
     *
     * The `gradientKernelSize` must be one of the following: 1, 3, 5, 7.
     * If `gradientKernelSize = 3`, the Scharr operator will be used to
     * calculate the gradient, otherwise the Sobel operator will be used.
     *
     * More information about the structure tensor can be found on
     * <a href="https://en.wikipedia.org/wiki/Structure_tensor"> Wikipedia</a>.
     *
     * @param voxelRadius Radius of subvolume used to calculate structure tensor
     * @param gradientKernelSize Size of the gradient kernel
     */
    StructureTensor structureTensorAt(
        int32_t vx,
        int32_t vy,
        int32_t vz,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const;

    /** @copydoc Volume::structureTensorAt() */
    StructureTensor structureTensorAt(
        const cv::Vec3i& index,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const
    {
        return structureTensorAt(
            index(0), index(1), index(2), voxelRadius, gradientKernelSize);
    }

    /**
     * @brief Compute the structure tensor for a subvoxel position
     *
     * @copydetails Volume::structureTensorAt()
     *
     * This version computes the structure tensor for subvoxel positions within
     * the volume.
     */
    StructureTensor interpolatedStructureTensorAt(
        double vx,
        double vy,
        double vz,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const;

    /** @copydoc interpolatedStructureTensorAt() */
    StructureTensor interpolatedStructureTensorAt(
        const cv::Vec3d& index,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const
    {
        return interpolatedStructureTensorAt(
            index(0), index(1), index(2), voxelRadius, gradientKernelSize);
    }

    /**
     * @brief Compute the eigenvalues and eigenvectors from the structure tensor
     * for a voxel position
     *
     * @copydetails structureTensorAt()
     */
    EigenPairs eigenPairsAt(
        int32_t x,
        int32_t y,
        int32_t z,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const;

    /** @copydoc eigenPairsAt() */
    EigenPairs eigenPairsAt(
        const cv::Vec3i& index,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const
    {
        return eigenPairsAt(
            index(0), index(1), index(2), voxelRadius, gradientKernelSize);
    }

    /**
     * @brief Compute the eigenvalues and eigenvectors from the structure tensor
     * for a subvoxel position
     *
     * @copydetails eigenPairsAt()
     *
     * This version computes the structure tensor for subvoxel positions within
     * the volume.
     */
    EigenPairs interpolatedEigenPairsAt(
        double x,
        double y,
        double z,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const;

    /** @copydoc interpolatedEigenPairsAt() */
    EigenPairs interpolatedEigenPairsAt(
        const cv::Vec3d& index,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const
    {
        return interpolatedEigenPairsAt(
            index(0), index(1), index(2), voxelRadius, gradientKernelSize);
    }

    /**
     * @brief Get an axis-aligned cuboid subvolume centered on a voxel
     * @param center Center position of the subvolume
     * @param rx Radius of the subvolume X-axis
     * @param ry Radius of the subvolume Y-axis
     * @param rz Radius of the subvolume Z-axis
     */
    template <typename DType>
    Tensor3D<DType> getVoxelNeighbors(
        const cv::Vec3i& center, int32_t rx, int32_t ry, int32_t rz) const
    {
        // Safety checks
        assert(
            center(0) >= 0 && center(0) < sliceWidth_ && center(1) >= 0 &&
            center(1) < sliceHeight_ && center(2) >= 0 &&
            center(2) < numSlices_ && "center must be inside volume");

        Tensor3D<DType> v(2 * rx + 1, 2 * ry + 1, 2 * rz + 1);
        for (int32_t k = center(2) - rz, c = 0; k <= center(2) + rz; ++k, ++c) {
            // If k index is out of bounds, then keep it at zeros and go on
            if (k < 0 || k >= numSlices_) {
                continue;
            }
            for (int32_t j = center(1) - ry, b = 0; j <= center(1) + ry;
                 ++j, ++b) {
                for (int32_t i = center(0) - rx, a = 0; i <= center(0) + rx;
                     ++i, ++a) {
                    if (i >= 0 && j >= 0 && i < sliceWidth_ &&
                        j < sliceHeight_) {
                        v(a, b, c) = DType(intensityAt(i, j, k));
                    }
                }
            }
        }

        return v;
    }

    /**
     * @brief Get an axis-aligned cube subvolume centered on a voxel
     * @param center Center position of the subvolume
     * @param radius Radius of the subvolume X, Y, and Z axes
     */
    template <typename DType>
    Tensor3D<DType> getVoxelNeighborsCubic(
        const cv::Vec3i& center, int32_t radius) const
    {
        return getVoxelNeighbors<DType>(center, radius, radius, radius);
    }

    /**
     * @brief Get an axis-aligned cuboid subvolume centered on a subvoxel
     *
     * @copydetails getVoxelNeighbors()
     */
    template <typename DType>
    Tensor3D<DType> getVoxelNeighborsInterpolated(
        const cv::Vec3d& center,
        int rx,
        int ry,
        int rz,
        const cv::Vec3d& xvec = {1, 0, 0},
        const cv::Vec3d& yvec = {0, 1, 0},
        const cv::Vec3d& zvec = {0, 0, 1}) const
    {
        if (!isInBounds(center)) {
            throw std::range_error("center not in bounds");
        }

        Tensor3D<DType> v(2 * rx + 1, 2 * ry + 1, 2 * rz + 1);
        for (int c = 0; c < 2 * rz + 1; ++c) {
            for (int b = 0; b < 2 * ry + 1; ++b) {
                for (int a = 0; a < 2 * rx + 1; ++a) {
                    auto xOffset = -rx + a;
                    auto yOffset = -ry + b;
                    auto zOffset = -rz + c;
                    auto p = center + (xvec * xOffset) + (yvec * yOffset) +
                             (zvec * zOffset);
                    v(a, b, c) = DType(interpolateAt(p));
                }
            }
        }

        return v;
    }

    /**
     * @brief Get an axis-aligned cube subvolume centered on a subvoxel
     *
     * @copydetails getVoxelNeighborsCubic()
     */
    template <typename DType>
    Tensor3D<DType> getVoxelNeighborsCubicInterpolated(
        const cv::Vec3d center, int32_t radius) const
    {
        return getVoxelNeighborsInterpolated<DType>(
            center, radius, radius, radius);
    }

    /**
     * @brief Get the subvoxels that intersect a ray projected from a center
     * subvoxel
     *
     * @param center Ray origin/center
     * @param majorAxis The direction of the ray
     * @param radius Length of the ray (in voxel units)
     * @param interval Sampling interval (in voxel units)
     * @param direction See volcart::Direction
     */
    Neighborhood getVoxelNeighborsLinearInterpolated(
        const cv::Vec3d& center,
        cv::Vec3d majorAxis,
        double radius,
        double interval,
        Direction direction = Direction::Bidirectional) const;
    /**@}*/

private:
    /** Directory containing the slice images */
    boost::filesystem::path path_;
    /** Volume metadata */
    volcart::Metadata metadata_;
    /** Number of slices in the Volume */
    int numSlices_;
    /** Width of the slice images */
    int sliceWidth_;
    /** Height of the slice images */
    int sliceHeight_;
    /** Number of characters in each slice filename. Used to account for
     * zero-padding. */
    int numSliceCharacters_;
    /** Slice image cache */
    mutable volcart::LRUCache<int32_t, cv::Mat> cache_;

    /** @brief Calculate the gradient of a subvolume */
    Tensor3D<cv::Vec3d> volume_gradient_(
        const Tensor3D<double>& v, int32_t gradientKernelSize) const;

    /**
     * @brief Calculate gradient of a 2D image along a specific axis
     *
     * Automatically selects the gradient operator using the provided kernel
     * size. If `kSize = 3`, the Scharr operator will be used, otherwise the
     * Sobel operator will be used.
     */
    cv::Mat_<double> gradient_(
        const cv::Mat_<double>& input, Axis axis, int32_t ksize) const;
};
}
