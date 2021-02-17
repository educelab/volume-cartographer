#pragma once

/** @file */

#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/Metadata.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart
{
/**
 * @class Texture
 * @author Seth Parker
 * @date 10/20/15
 *
 * @brief Holds the results of a texturing algorithm
 *
 * A Texture is, generically, the result of running a texturing algorithm. It
 * typically is a single image, but can be also be a set of images. Since the
 * results of texturing are highly dependent upon the choice of algorithm and
 * input parameters to that algorithm, this class provides a mechanism for
 * grouping the results and their associated metadata into a single data
 * structure.
 *
 * @ingroup Types
 */
class Texture
{
public:
    /**@{*/
    /** @brief Default constructor */
    Texture();

    /** @brief Open a Texture folder from disk */
    explicit Texture(volcart::filesystem::path path);
    /**@}*/

    /**@{*/
    /** @brief Get the Texture Metadata */
    volcart::Metadata metadata() const { return metadata_; }

    /** @brief Get the Texture ID */
    std::string id() const { return metadata_.get<std::string>("id"); }

    /** @brief Get the width of Texture image(s) */
    int width() const { return width_; }

    /** @brief Get the height of Texture image(s) */
    int height() const { return height_; }

    /** @brief Get the number of Texture images */
    size_t numberOfImages() const { return images_.size(); }

    /** @brief Return whether the Texture has images */
    bool hasImages() const { return !images_.empty(); }

    /** @brief Return whether the Texture has an initialized PerPixelMap */
    bool hasMap() const { return ppm_.initialized(); }
    /**@}*/

    /**@{*/
    /** @brief Get a Texture image by index number */
    cv::Mat image(size_t id) const { return images_[id]; }

    /**
     * @brief Add an image to the Texture
     *
     * @return The index number of the image
     */
    size_t addImage(cv::Mat image);

    /**
     * @brief Assign an image to an index within the Texture
     *
     * @throws std::out_of_range If `id` is not in the range of image indices
     */
    void setImage(size_t id, cv::Mat m);

    /** @brief Get the intensity value for an ITKPoint by ID
     *
     * Will return volcart::TEXTURE_NO_VALUE if the ITKPoint does not have a
     * valid UV mapping or if the UV map is empty.
     */
    double intensity(int pointId, int imageId = 0);
    /**@}*/

    /**@{*/
    /** @brief Set the PerPixelMap */
    void setPPM(const PerPixelMap& m) { ppm_ = m; }

    /** @brief Get the PerPixelMap */
    const PerPixelMap& ppm() const { return ppm_; }

    /** @copydoc ppm() const */
    PerPixelMap& ppm() { return ppm_; }

    /** @brief Get the pixel mask
     *
     * @see PerPixelMap::setMask()
     */
    cv::Mat mask() { return ppm_.mask(); }

    /** @brief Get the UVMap */
    const volcart::UVMap& uvMap() const { return ppm_.uvMap(); }

    /** @copydoc uvMap() const */
    volcart::UVMap& uvMap() { return ppm_.uvMap(); }
    /**@}*/

    /** Defines an empty texture to be -1 */
    constexpr static double NO_VALUE = -1.0;

private:
    /** Texture metadata */
    volcart::Metadata metadata_;
    /** Location for the Texture on disk */
    volcart::filesystem::path path_;
    /** Width of the Texture image(s) */
    int width_;
    /** Height of the Texture image(s) */
    int height_;
    /** Image data */
    std::vector<cv::Mat> images_;
    /** PerPixelMap from which this Texture was generated */
    PerPixelMap ppm_;
};
}  // namespace volcart
