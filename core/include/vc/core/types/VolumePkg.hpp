#pragma once

#include <iostream>
#include <map>

#include <boost/filesystem.hpp>

#include "vc/core/types/Metadata.hpp"
#include "vc/core/types/Render.hpp"
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
    VolumePkg(const boost::filesystem::path& fileLocation, int version);

    /**
     * @brief Construct a VolumePkg from a .volpkg file stored at
     * `fileLocation.`
     * @param fileLocation The root of the VolumePkg file
     */
    explicit VolumePkg(const boost::filesystem::path& fileLocation);

    /** VolumePkg shared pointer */
    using Pointer = std::shared_ptr<VolumePkg>;

    /**
     * @copybrief VolumePkg(boost::filesystem::path fileLocation, int version)
     *
     * Returns a shared pointer to the VolumePkg.
     */
    static Pointer New(boost::filesystem::path fileLocation, int version);

    /**
     * @copybrief VolumePkg(boost::filesystem::path fileLocation)
     *
     * Returns a shared pointer to the VolumePkg.
     */
    static Pointer New(boost::filesystem::path fileLocation);
    /**@}*/

    /** @name Metadata */
    /**@{*/
    /**
     * @brief Returns the identifying name of the VolumePkg.
     * @return Name of the VolumePkg
     */
    std::string name() const;

    /**
     * @brief Returns the VolumePkg version.
     *
     * Use in conjunction with volcart::VERSION_LIBRARY to verify the presence
     * of
     * specific VolumePkg metadata keys.
     *
     * @return Version number of VolumePkg
     */
    int version() const;

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
    double materialThickness() const;

    /*
    std::string scanner() const;
    double objToSource() const;
    double cameraToSource() const;
    // in kV
    int sourceVoltage() const;
    // in uA
    int sourceCurrent() const;
    // in bits
    int depth() const;
    // in ms
    int exposureTime() const;
    double rotationStep() const;
    int connectedScanNum() const;
    // how to use DateTime.hpp class?
    int studyDateTime() const;
    int scanDuration() const;

    std::string datasetPrefix() const;
    // how to use DateTime.hpp class?
    int dateTime() const;
    std::string resultFileType() const;
    // in bytes
    int resultHeaderLength() const;
    // in pixels
    int resultImgWidth() const;
    // in um
    float pixelSize() const;

    // all in pixels
    int topROI() const;
    int bottomROI() const;
    int leftROI() const;
    int rightROI() const;
    int referenceLength() const;

    int filenameIndexLength() const;
    std::string filenamePrefix() const;
     */

    /** @brief Return the VolumePkg Metadata */
    Metadata metadata() const { return config_; }

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
    void saveMetadata() { config_.save(); }

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
    const Volume::Pointer volume() const;

    /** @copydoc volume() */
    Volume::Pointer volume();

    /** @brief Get a Volume by uuid */
    const Volume::Pointer volume(const Volume::Identifier& id) const;

    /** @copydoc VolumePkg::volume(std::string) const */
    Volume::Pointer volume(const Volume::Identifier& id);
    /**@}*/

    /** @name Segmentation Data */
    /**@{*/
    /** @brief Return whether there are Segmentations */
    bool hasSegmentations() { return !segmentations_.empty(); }

    /** @brief Get the number of Segmentations */
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

    /** @name Render Data */
    /**@{*/
    /** @brief Return whether there are Renders */
    bool hasRenders() { return !renders_.empty(); }

    /** @brief Get the number of Renders */
    size_t numberOfRenders() { return renders_.size(); }

    /** @brief Get the list of Render IDs */
    std::vector<Render::Identifier> renderIDs() const;

    /** @brief Get the list of Render names */
    std::vector<std::string> renderNames() const;

    /**
     * @brief Creates a new Render.
     *
     * Populates the .volpkg file with a new Render directory and adds the ID to
     * the internal list of Renders.
     */
    Render::Pointer newRender(std::string name = "");

    /** @brief Get a Render by uuid */
    const Render::Pointer render(const Render::Identifier& id) const
    {
        return renders_.at(id);
    }

    /** @copydoc VolumePkg::render(std::string) const */
    Render::Pointer render(const Render::Identifier& id)
    {
        return renders_.at(id);
    }
    /**@}*/

private:
    /** VolumePkg metadata */
    Metadata config_;
    /** The root directory of the VolumePkg */
    boost::filesystem::path rootDir_;
    /** The subdirectory containing Volume data */
    boost::filesystem::path volsDir_;
    /** The subdirectory containing Segmentation data */
    boost::filesystem::path segsDir_;
    /** The subdirectory containing Render data */
    boost::filesystem::path rendDir_;
    /** The list of all Volumes in the VolumePkg. */
    std::map<Volume::Identifier, Volume::Pointer> volumes_;
    /** The list of all Segmentations in the VolumePkg. */
    std::map<Segmentation::Identifier, Segmentation::Pointer> segmentations_;
    /** The list of all Renders in the VolumePkg. */
    std::map<Render::Identifier, Render::Pointer> renders_;

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