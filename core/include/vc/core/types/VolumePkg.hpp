#pragma once

#include <iostream>
#include <map>

#include <boost/filesystem.hpp>

#include "vc/core/types/Metadata.hpp"
#include "vc/core/types/Segmentation.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/types/VolumePkgVersion.hpp"

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
    /** @brief Return whether there are Volumes */
    bool hasVolumes() { return !volumes_.empty(); }

    /** @brief Get the number of Volumes */
    size_t numberOfVolumes() { return volumes_.size(); }

    /** @brief Get the list of volume IDs */
    std::vector<Volume::Identifier> volumeIDs() const;

    /** @brief Get the list of volumes names */
    std::vector<std::string> volumeNames() const;

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

    /** @brief Get a Volume by uuid */
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
    /** @brief Return whether there are Volumes */
    bool hasSegmentations() { return !segmentations_.empty(); }

    /** @brief Get the number of Volumes */
    size_t numberOfSegmentations() { return segmentations_.size(); }

    /** @brief Get the list of Segmentation IDs */
    std::vector<Segmentation::Identifier> segmentationIDs() const;

    /** @brief Get the list of Segmentation names */
    std::vector<std::string> segmentationNames() const;

    /**
     * @brief Creates a new segmentation.
     *
     * Populates the .volpkg file with a new segmentation directory and adds the
     * ID to the internal list of segmentations.
     * @return Identifier name of the new segmentation
     */
    Segmentation::Pointer newSegmentation(std::string name = "");

    /** @brief Get a Segmentation by uuid */
    const Segmentation::Pointer segmentation(
        const Segmentation::Identifier& id) const
    {
        return segmentations_.at(id);
    }

    /** @copydoc VolumePkg::segmentation(std::string) const */
    Segmentation::Pointer segmentation(const Segmentation::Identifier& id)
    {
        return segmentations_.at(id);
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
    /** The list of all segmentations in the VolumePkg. */
    std::map<Segmentation::Identifier, Segmentation::Pointer> segmentations_;

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