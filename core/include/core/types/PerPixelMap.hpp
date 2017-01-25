#pragma once

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>

#include "core/types/OrderedPointSet.hpp"
#include "core/types/UVMap.hpp"

namespace volcart
{
/**
 * @class PerPixelMap
 * @author Seth Parker
 * @date 3/17/16
 *
 * @brief A raster of a UV Map for reverse lookups
 *
 * Effectively a raster of a UV Map for reverse lookups.
 * Every pixel in the map holds the 3D position and normal
 * used to generate that pixel's intensity in the corresponding
 * texture image. Useful for regenerating textures with differing
 * parameters or for identifying where in a volume a particular
 * came from.
 *
 *
 * @ingroup Core
 */
class PerPixelMap
{
public:
    /** @name Constructors*/
    //@{
    /** @brief Creates an empty PerPixelMap */
    PerPixelMap() : width_{0}, height_{0} { initialize_map_(); }

    /**
     * @brief Creates a new PerPixelMap with a set size
     * @param height How many pixels high the map will be
     * @param width How many pixels wide the map will be
     *
     * Initializes a map of a specific height and width
     * with all zeros.
     */
    PerPixelMap(size_t height, size_t width) : width_{width}, height_{height}
    {
        initialize_map_();
    }
    //@}

    /**
     * @brief Performs a check to see if map is initialzed
     *
     * Checks the map height and width to see if it is what
     * we expect. It also checks to make sure the height
     * and width are not zero.
     */
    bool initialized() const
    {
        return width_ == map_.width() && height_ == map_.height() &&
               width_ > 0 && height_ > 0;
    }

    /**
     * @brief Returns the texture information stored at the point(x,y)
     *
     * This function takes in the operands in the reverse order
     * to work with OpenCV standards however OrderedPointSet
     * uses the xy format so the operands are switched so the
     * correct point is fetched.
     *
     * @see core/types/OrderedPointSet.hpp
     */
    cv::Vec6d& operator()(size_t y, size_t x) { return map_(x, y); }

    /** @name Metadata */
    //@{
    /** @brief Sets the dimensions of the map*/
    void setDimensions(size_t h, size_t w);
    /** @brief Sets the width of the map*/
    void setWidth(size_t w);
    /** @brief Sets the height of the map*/
    void setHeight(size_t h);
    /** @brief Returns the width of the map*/
    int width() const { return width_; }
    /** @brief Returns the height of the map*/
    int height() const { return height_; }

    /**
     * @brief Sets the UVMap to the given UVMap
     * @see core/types/UVMap.hpp
     */
    void setUVMap(const UVMap& u) { uvMap_ = u; }
    /**
     * @brief Returns the UVMap that has been set
     * @see core/types/UVMap.hpp
     */
    const UVMap& uvMap() const { return uvMap_; }
    /**
     * @brief Returns the location of the UVMap
     * @see core/types/UVMap.hpp
     */
    UVMap& uvMap() { return uvMap_; }

    /** @brief Sets the Mask */
    void setMask(const cv::Mat& m) { mask_ = m.clone(); }
    /** @brief Returns the Mask*/
    cv::Mat mask() const { return mask_; }
    /** @brief Returns a copy of the Mask*/
    cv::Mat maskCopy() const { return mask_.clone(); }
    /**@brief Return whether or not there is a mapping for the pixel at x,y */
    bool hasMapping(size_t y, size_t x)
    {
        return mask_.at<uint8_t>(y, x) == 255;
    }
    //@}

    /** @name Disk IO*/
    //@{
    /** @brief Writes the given PerPixelMap out to disk*/
    static void WritePPM(
        const boost::filesystem::path& path, const PerPixelMap& map);
    /** @brief Reads in a PerPixelMap from a given file*/
    static PerPixelMap ReadPPM(const boost::filesystem::path& path);
    //@}

private:
    /**
     * @brief Initializes the map
     * Checks to see that the height and widith have been set and
     * initializes a map of that height and width with all zeros
     */
    void initialize_map_();
    /** Width of the map*/
    size_t width_;
    /** Height of the map*/
    size_t height_;
    /** OrderedPointSet where the map is stored*/
    volcart::OrderedPointSet<cv::Vec6d> map_;
    /** @brief Mask for the image
     *
     * The mask stores either 255 or 0 depending
     * on if the PerPixelMap has a valid entry for a particular pixel.
     * If there is a valid number stored for a pixel in the PerPixelMap
     * then the mask stores 255, otherwise it stores 0.
     *
     * @newline
     *
     * This can also be used to make texture images with
     * transparent backgrounds.
     */
    cv::Mat mask_;
    /** uvMap for the image
     * @see core/types/UVMap.hpp
     */
    UVMap uvMap_;
};
}  // namespace volcart
