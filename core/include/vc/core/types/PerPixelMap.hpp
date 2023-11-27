#pragma once

/** @file */

#include <cstddef>
#include <memory>

#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/UVMap.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart
{
/**
 * @class PerPixelMap
 * @author Seth Parker
 * @date 3/17/16
 *
 * @brief A raster of a UVMap that provides a per-pixel mapping between a Volume
 * and a Texture generated from that volume
 *
 * After a segmentation mesh is flattened, the resulting UV space is sampled at
 * a specific resolution in order to generate a texture space (e.g. image). A
 * texturing algorithm is responsible for filtering information from the volume
 * and placing it into this space, the intended result being an image of a
 * manuscript's text. The transformation that maps 2D coordinates in texture
 * space to 3D coordinates in volume space is defined by the per-vertex
 * transformation generated by flattening, however, there are numerous ways that
 * the points \em between vertices can be mapped back into the volume. Since the
 * calculation of this mapping can be expensive, it is often desirable to
 * perform this operation only once.
 *
 * The PerPixelMap (PPM) provides a method for storing the result of this
 * calculation. It has the same dimensions as texture space, and each pixel
 * holds the mapped, 3D position for that pixel in volume space. It additionally
 * holds 3 extra `double` elements, typically used to store the surface normal
 * vector for the 3D position (calculated from the segmentation mesh).
 *
 * The position and normal vector are stored in a `cv::Vec6d`:
 * `{x, y, z, nx, ny, nz}`.
 *
 * The texturing::PPMGenerator class generates a PerPixelMap by mapping
 * pixels through the barycentric coordinates of the mesh's triangular faces.
 *
 * @ingroup Types
 */
class PerPixelMap
{
public:
    /** Convenience structure for a single pixel's mapping information */
    struct PixelMap {
        /** Default constructor */
        PixelMap() = default;

        /** Construct and initialize */
        PixelMap(std::size_t x, std::size_t y, cv::Vec6d value);

        /** PPM Pixel position X */
        std::size_t x{0};
        /** PPM Pixel position Y */
        std::size_t y{0};
        /** Mapped Volume position */
        cv::Vec3d pos;
        /** Surface normal at mapped Volume position */
        cv::Vec3d normal;
    };

    /** Pointer type */
    using Pointer = std::shared_ptr<PerPixelMap>;

    /**@{*/
    /** @brief Default constructor */
    PerPixelMap() = default;

    /** @brief Constructor with width and height parameters */
    PerPixelMap(std::size_t height, std::size_t width);

    /** Static New function for all constructors of T */
    template <typename... Args>
    static auto New(Args... args) -> Pointer
    {
        return std::make_shared<PerPixelMap>(std::forward<Args>(args)...);
    }

    /**
     * @brief Return whether the PerPixelMap has been initialized
     *
     * The map is initialized as soon as its width and height have been set.
     */
    [[nodiscard]] auto initialized() const -> bool;
    /**@}*/

    /**@{*/
    /** @brief Get the mapping for a pixel by x, y coordinate */
    auto operator()(std::size_t y, std::size_t x) const -> const cv::Vec6d&;

    /** @copydoc operator()() */
    auto operator()(std::size_t y, std::size_t x) -> cv::Vec6d&;

    /** @copydoc operator()() */
    [[nodiscard]] auto getMapping(std::size_t y, std::size_t x) const
        -> const cv::Vec6d&;

    /** @copydoc operator()() */
    auto getMapping(std::size_t y, std::size_t x) -> cv::Vec6d&;

    /**
     * @brief Return whether there is a mapping for the pixel at x, y
     *
     * Returns `true` is the pixel mask has not been set or is empty
     */
    [[nodiscard]] auto hasMapping(std::size_t y, std::size_t x) const -> bool;

    /** @brief Get the mapping for a pixel as a PixelMap */
    auto getAsPixelMap(std::size_t y, std::size_t x) -> PixelMap;

    /**
     * @brief Get all valid pixel mappings as a list of PixelMap
     *
     * Uses hasMapping() to determine which pixels in the PPM are valid.
     */
    [[nodiscard]] auto getMappings() const -> std::vector<PixelMap>;
    /**@}*/

    /**@{*/
    /**
     * @brief Set the dimensions of the map
     *
     * @warning Changing the size of the PerPixelMap will clear it of data.
     * Setting either dimension to 0 will result in undefined behavior.
     */
    void setDimensions(std::size_t h, std::size_t w);

    /**
     * @brief Set the width of the map
     * @copydetails setDimensions()
     */
    void setWidth(std::size_t w);

    /**
     * @brief Set the height of the map
     * @copydetails setDimensions()
     */
    void setHeight(std::size_t h);

    /** @brief Get the width of the map */
    [[nodiscard]] auto width() const -> std::size_t;

    /** @brief Get the height of the map */
    [[nodiscard]] auto height() const -> std::size_t;
    /**@}*/

    /**@{*/
    /** @brief Get the pixel mask */
    [[nodiscard]] auto mask() const -> cv::Mat;

    /**
     * @brief Set the pixel mask
     *
     * If the pixel mask is not set or is empty, every pixel is assumed to have
     * a mapping.
     *
     * Not every pixel in the PerPixelMap will have a mapped value. The pixel
     * mask is an 8bpc, single channel image that indicates which pixels do and
     * do not have mappings. 0 = No mapping, 255 = Has mapping.
     */
    void setMask(const cv::Mat& m);

    /**
     * @brief Get the cell map image
     *
     * The cell map contains the face assignment for each pixel in the PPM.
     * This is an optional feature, and older PPMs may not make this information
     * available.
     */
    [[nodiscard]] auto cellMap() const -> cv::Mat;

    /**
     * @brief Set the cell map image
     *
     * @copydetails cellMap()
     */
    void setCellMap(const cv::Mat& m);
    /**@}*/

    /** @brief Return whether this PerPixelMap is associated with a Volume */
    [[nodiscard]] auto hasVolumeID() const -> bool;

    /** @brief Get the ID of the Volume associated with this PerPixelMap */
    [[nodiscard]] auto getVolumeID() const -> Volume::Identifier;

    /** @brief Set the ID of the Volume associated with this PerPixelMap */
    void setVolumeID(const Volume::Identifier& id);

    /**@{*/
    /** @brief Write a PerPixelMap to disk */
    static void WritePPM(const filesystem::path& path, const PerPixelMap& map);

    /** @brief Read a PerPixelMap from disk */
    static auto ReadPPM(const filesystem::path& path) -> PerPixelMap;
    /**@}*/

private:
    /**
     * Initialize the map for value assignment
     *
     * Does nothing if either the height or width are 0.
     */
    void initialize_map_();

    /** Height of the map */
    std::size_t height_{0};
    /** Width of the map */
    std::size_t width_{0};
    /** Map data storage */
    OrderedPointSet<cv::Vec6d> map_;

    /**
     * Pixel mask
     *
     * The pixel mask is an 8bpc, single channel image that indicates which
     * pixels do and do not have mappings. 0 = No mapping, 255 = Has mapping.
     */
    cv::Mat mask_;

    /** Cell map */
    cv::Mat cellMap_;
};
}  // namespace volcart
