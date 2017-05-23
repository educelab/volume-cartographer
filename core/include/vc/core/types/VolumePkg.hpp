#pragma once

#include <cstdlib>
#include <iostream>
#include <map>

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/Texture.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/types/VolumePkgVersion.hpp"
#include "vc/external/json.hpp"

namespace volcart
{

/**
 * @class VolumePkg
 * @brief The interface to the VolumePkg (.volpkg) file format.
 *
 * Provides access to volume, segmentation, and rendering data stored on disk.
 *
 * @warning VolumePkg is not thread safe.
 *
 * @ingroup Types
 * @ingroup VolumePackage
 *
 * @see apps/src/packager.cpp
 *      apps/src/metadata.cpp
 *      examples/src/volpkg.cpp
 *      examples/src/ResliceAnalysis.cpp
 */
class VolumePkg
{
public:
    /**@{*/
    /**
     * @brief Construct an empty VolumePkg of a specific version number.
     *
     * This will construct an empty VolumePkg in memory and set its expected
     * location on disk. Note: You must call initialize() before the file can
     * be written to and accessed. Only metadata keys may be modified before
     * initialize is called.
     *
     * @param fileLocation The location to store the VolPkg
     * @param version Version of VolumePkg you wish to construct
     */
    VolumePkg(boost::filesystem::path fileLocation, int version);

    /**
     * @brief Construct a VolumePkg from a .volpkg file stored at
     * `fileLocation.`
     * @param fileLocation The root of the VolumePkg file
     */
    explicit VolumePkg(boost::filesystem::path fileLocation);

    /** VolumePkg shared pointer */
    using Pointer = std::shared_ptr<VolumePkg>;

    /** @copybrief VolumePkg(boost::filesystem::path fileLocation)
     *
     * Returns a shared pointer to the VolumePkg.
     */
    static Pointer New(boost::filesystem::path fileLocation);
    /**@}*/

    /**@{*/
    /**
     * @brief Prints the JSON object that stores VolumePkg metadata. Debug only.
     */
    void printJSON() const { config_.printObject(); }

    /**
     * @brief Prints the paths to important VolumePkg subdirectories.
     * Debug only.
     */
    void printDirs() const
    {
        std::cout << "root: " << rootDir_ << " seg: " << segsDir_
                  << " slice: " << volsDir_ << std::endl;
    }
    /**@}*/

    /** @name Metadata */
    /**@{*/
    /**
     * @brief Returns the identifying name of the VolumePkg.
     * @return Name of the VolumePkg
     */
    std::string getPkgName() const;

    /**
     * @brief Returns the VolumePkg version.
     *
     * Use in conjunction with volcart::VERSION_LIBRARY to verify the presence
     * of
     * specific VolumePkg metadata keys.
     *
     * @return Version number of VolumePkg
     */
    int getVersion() const;

    /**
     * @brief Sets the value of `key` in the VolumePkg metadata.
     *
     * These values are stored only in memory until saveMetadata() is called.
     *
     * @param key Metadata key identifier
     * @param value Value to be stored
     */
    template <typename T>
    void setMetadata(const std::string& key, T value)
    {
        config_.set<T>(key, value);
    }

    /**
     * @brief Saves the metadata to the VolumePkg (.volpkg) file.
     */
    void saveMetadata() { config_.save(rootDir_ / "config.json"); }

    /**
     * @brief Saves the metadata to a user-specified location.
     * @param filePath Path to output file
     */
    void saveMetadata(const boost::filesystem::path& filePath)
    {
        config_.save(filePath);
    }
    /**@}*/

    /** @name Volume Data */
    /**@{*/
    /** @brief Get the number of Volumes */
    size_t numberOfVolumes() { return volumes_.size(); }

    /** @brief Get the list of volumes */
    std::vector<Volume::Description> volumes() const;

    /**
     * @brief Add a new Volume to the VolumePkg
     * @param name Human-readable name for the new Volume. Defaults to the
     * auto-generated Volume ID.
     * @return Pointer to the new Volume
     */
    Volume::Pointer newVolume(std::string name = "");

    /** @brief Get the first Volume */
    const Volume::Pointer volume() const { return volumes_.begin()->second; }

    /** @copydoc volume() */
    Volume::Pointer volume() { return volumes_.begin()->second; }

    /** @brief Get a Volume by index number */
    const Volume::Pointer volume(const Volume::Identifier& id) const
    {
        return volumes_.at(id);
    }

    /** @copydoc VolumePkg::volume(std::string) const */
    Volume::Pointer volume(const Volume::Identifier& id)
    {
        return volumes_.at(id);
    }

    /**
     * @brief Returns the width of the slice images.
     *
     * This number is retrieved from the metadata and is not validated
     * against the slices stored in the .volpkg file.
     */
    int getSliceWidth() const;

    /**
     * @brief Returns the height of the slice images.
     *
     * This number is retrieved from the metadata and is not validated
     * against the slices stored in the .volpkg file.
     */
    int getSliceHeight() const;

    /**
     * @brief Returns the number of slice images.
     *
     * This number is retrieved from the metadata and is not validated
     * against the slices stored in the .volpkg file.
     */
    int getNumberOfSlices() const;

    /**
     * @brief Returns the size of voxels in microns (um).
     *
     * This is the "real-world" size of voxels. Only isometric voxels (voxels
     * with equal edge lengths) are supported.
     * @return Voxel size in microns (um)
     */
    double getVoxelSize() const;

    /**
     * @brief Returns the approx. thickness of a material layer in microns (um).
     *
     * This value is approximated by the user when the VolumePkg is created.
     * This is an intrinsic property of the scanned object and is therefore
     * indepedent of scan resolution. The material thickness in microns can be
     * used to estimate the material thickness in voxels for scans of any
     * resolution.
     *
     * \f[
        \frac{\mbox{Material Thickness }(um)}{\mbox{Voxel Size }(um)}
        = \mbox{Material Thickness }(voxels)
      \f]
     *
     * @return Layer thickness, measured in microns (um).
     */
    double getMaterialThickness() const;
    /**@}*/

    /** @name Segmentation Data */
    /**@{*/
    /**
     * @brief Creates a new segmentation.
     *
     * Populates the .volpkg file with a new segmentation directory and adds the
     * ID to the internal list of segmentations.
     * @return Identifier name of the new segmentation
     */
    std::string newSegmentation();

    /**
     * @brief Returns the list of Segmentation IDs for the VolumePkg.
     *
     * IDs in this list can be passed to setActiveSegmentation() in order to
     * access data from a specific segmentation.
     * @return List of segmentation IDs
     */
    std::vector<std::string> getSegmentations() const;

    /**
     * @brief Sets the active segmentation.
     *
     * Data access functions like openCloud() and getMesh() return data from the
     * active segmentation. To get data from other segmentations, you must first
     * change the active segmentation using this function.
     *
     * @param id Segmentation name of desired active segmentation
     */
    void setActiveSegmentation(const std::string& id);

    /**
     * @brief Returns the ID of the active segmentation.
     * @return Segmentation ID of active segmentation
     */
    std::string getActiveSegmentation();

    /**
     * @brief Returns the directory path for the active segmentation.
     *
     * This path can be absolute or relative.
     *
     * @return Directory path to the active segmentation
     */
    boost::filesystem::path getActiveSegPath();

    /**
     * @brief Returns the OrderedPointSet for the active segmentation.
     *
     * This returns a point cloud that represents segmented surface points
     * within the Volume. An OrderedPointSet provides 2D access to these points.
     *
     * @return Segmented surface as an OrderedPointSet
     */
    OrderedPointSet<cv::Vec3d> openCloud() const;

    /**
     * @brief Saves an OrderedPointSet for the active segmentation to the
     * .volpkg file.
     *
     * Saves the points in `ps` to the active segmentation's subdirectory in the
     * .volpkg file. Throws volcart::IOException on write failure. Otherwise
     * returns integer success code.
     * @warning Data currently saved in the active segmentation's directory will
     * be overwritten.
     *
     * @param ps PointSet to be saved to the .volpkg file.
     * @return `EXIT_SUCCESS`
     */
    int saveCloud(const OrderedPointSet<cv::Vec3d>& ps) const;
    /**@}*/

    /** @name Render Data */
    /**@{*/
    /**
     * @brief Returns the file path of the meshed segmentation data.
     *
     * Returns the file path to `cloud.ply`, the meshed representation of the
     * segmented OrderedPointSet. Does not validate that this file exists.
     * @return File path to segmentation mesh
     */
    boost::filesystem::path getMeshPath() const;

    /**
     * @brief Saves `mesh` to the active * segmentation's subdirectory in the
     * .volpkg file.
     *
     * Saves a volcart::ITKMesh to the volpkg file
     * @warning Data currently saved in the active segmentation's directory will
     * be overwritten.
     *
     * @param mesh PointSet to be saved to the .volpkg file.
     * @return `EXIT_SUCCESS`
     */
    int saveMesh(const ITKMesh::Pointer& mesh) const;

    /**
     * @brief Saves the provided mesh and texture information active
     * segmentation's subdirectory in the .volpkg file.
     *
     * Writes a texture-mapped OBJ file to `textured.{obj|mtl|png}`.
     * volcart::Texture object should be populated with a UVMap and at least one
     * texture image. This function will save only the first texture image.
     * @warning Data currently saved in the active segmentation's directory will
     * be overwritten.
     *
     * @param mesh The mesh imfornation to be saved
     * @param texture Populated Texture object
     * @see volcart::Texture
     */
    void saveMesh(const ITKMesh::Pointer& mesh, const Texture& texture) const;

    /**
     * @brief Returns the texture image saved in the active segmentation's
     * subdirectory in the .volpkg file.
     *
     * Returns an empty `cv::Mat` if file does not exist or could not be read.
     * @return A cv::Mat containing the image data
     */
    cv::Mat getTextureData() const;

    /**
     * @brief Saves a texture image to the active segmentation's subdirectory in
     * the .volpkg file.
     *
     * File is written to `{name}.png`.
     * @warning Data currently saved in the active segmentation's directory will
     * be overwritten.
     *
     * @param texture Texture image data
     * @param name Filename w/o extension [Default: "textured"]
     */
    void saveTextureData(
        const cv::Mat& texture, const std::string& name = "textured");

    /**
     * @brief Saves a texture image to the active segmentation's subdirectory in
     * the .volpkg file.
     *
     * Writes the image stored in `texture.image(index)`. File is written
     * to `textured.png` in the active segmentation's subdirectory.
     * @param texture Populated Texture object
     * @param index The index of the desired image in `texture`'s image array
     * [Default: 0]
     */
    void saveTextureData(const Texture& texture, int index = 0)
    {
        saveTextureData(texture.image(index));
    }
    /**@}*/

private:
    /** VolumePkg metadata */
    Metadata config_;
    /** The root directory of the VolumePkg */
    boost::filesystem::path rootDir_;
    /** The subdirectory containing Segmentation data */
    boost::filesystem::path segsDir_;
    /** The subdirectory containing slice data */
    boost::filesystem::path volsDir_;
    /** The list of all volumes in the VolumePkg. */
    std::map<Volume::Identifier, Volume::Pointer> volumes_;
    /** Segmentation ID of the segmentation that is currently being worked on */
    std::string activeSeg_;
    /** The list of all segmentations in the VolumePkg */
    std::vector<std::string> segmentations_;

    /**
     * @brief Populates an empty VolumePkg::config from a volcart::Dictionary
     * template
     *
     * The configuration is populated with all keys found in `dict`. This is not
     * validated against what is expected for the passed `version` number.
     * @param dict Metadata template
     * @param version Version number of the passed Dictionary
     * @return volcart::Metadata populated with default keys
     */
    static Metadata InitConfig(const Dictionary& dict, int version);
};
}