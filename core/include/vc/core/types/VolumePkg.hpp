#pragma once

/** @file */

#include <iostream>
#include <map>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/Metadata.hpp"
#include "vc/core/types/Render.hpp"
#include "vc/core/types/Segmentation.hpp"
#include "vc/core/types/Transforms.hpp"
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
    VolumePkg(const filesystem::path& fileLocation, int version);

    /**
     * @brief Construct a VolumePkg from a .volpkg file stored at
     * `fileLocation.`
     * @param fileLocation The root of the VolumePkg file
     */
    explicit VolumePkg(const filesystem::path& fileLocation);

    /** VolumePkg shared pointer */
    using Pointer = std::shared_ptr<VolumePkg>;

    /**
     * @copybrief VolumePkg(filesystem::path fileLocation, int version)
     *
     * Returns a shared pointer to the VolumePkg.
     */
    static auto New(filesystem::path fileLocation, int version) -> Pointer;

    /**
     * @copybrief VolumePkg(filesystem::path fileLocation)
     *
     * Returns a shared pointer to the VolumePkg.
     */
    static auto New(filesystem::path fileLocation) -> Pointer;
    /**@}*/

    /** @name Metadata */
    /**@{*/
    /**
     * @brief Returns the identifying name of the VolumePkg.
     * @return Name of the VolumePkg
     */
    [[nodiscard]] auto name() const -> std::string;

    /**
     * @brief Returns the VolumePkg version.
     *
     * Use in conjunction with volcart::VERSION_LIBRARY to verify the presence
     * of
     * specific VolumePkg metadata keys.
     *
     * @return Version number of VolumePkg
     */
    [[nodiscard]] auto version() const -> int;

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
    [[nodiscard]] auto materialThickness() const -> double;

    /** @brief Return the VolumePkg Metadata */
    [[nodiscard]] auto metadata() const -> Metadata;

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
    void saveMetadata();

    /**
     * @brief Saves the metadata to a user-specified location.
     * @param filePath Path to output file
     */
    void saveMetadata(const filesystem::path& filePath);
    /**@}*/

    /** @name Volume Data */
    /**@{*/
    /** @brief Return whether there are Volumes */
    auto hasVolumes() -> bool;

    auto hasVolume(const Volume::Identifier& id) const -> bool;

    /** @brief Get the number of Volumes */
    auto numberOfVolumes() -> size_t;

    /** @brief Get the list of volume IDs */
    [[nodiscard]] auto volumeIDs() const -> std::vector<Volume::Identifier>;

    /** @brief Get the list of volumes names */
    [[nodiscard]] auto volumeNames() const -> std::vector<std::string>;

    /**
     * @brief Add a new Volume to the VolumePkg
     * @param name Human-readable name for the new Volume. Defaults to the
     * auto-generated Volume ID.
     * @return Pointer to the new Volume
     */
    auto newVolume(std::string name = "") -> Volume::Pointer;

    /** @brief Get the first Volume */
    [[nodiscard]] auto volume() const -> const Volume::Pointer;

    /** @copydoc volume() const */
    auto volume() -> Volume::Pointer;

    /** @brief Get a Volume by uuid */
    [[nodiscard]] auto volume(const Volume::Identifier& id) const
        -> const Volume::Pointer;

    /** @copydoc VolumePkg::volume(const Volume::Identifier&) const */
    auto volume(const Volume::Identifier& id) -> Volume::Pointer;
    /**@}*/

    /** @name Segmentation Data */
    /**@{*/
    /** @brief Return whether there are Segmentations */
    auto hasSegmentations() -> bool;

    /** @brief Get the number of Segmentations */
    auto numberOfSegmentations() -> size_t;

    /** @brief Get the list of Segmentation IDs */
    [[nodiscard]] auto segmentationIDs() const
        -> std::vector<Segmentation::Identifier>;

    /** @brief Get the list of Segmentation names */
    [[nodiscard]] auto segmentationNames() const -> std::vector<std::string>;

    /**
     * @brief Creates a new segmentation.
     *
     * Populates the .volpkg file with a new segmentation directory and adds the
     * ID to the internal list of segmentations.
     * @return Identifier name of the new segmentation
     */
    auto newSegmentation(std::string name = "") -> Segmentation::Pointer;

    /** @brief Get a Segmentation by uuid */
    [[nodiscard]] auto segmentation(const Segmentation::Identifier& id) const
        -> const Segmentation::Pointer;

    /** @copydoc VolumePkg::segmentation(const Segmentation::Identifier&) const
     */
    auto segmentation(const Segmentation::Identifier& id)
        -> Segmentation::Pointer;
    /**@}*/

    /** @name Render Data */
    /**@{*/
    /** @brief Return whether there are Renders */
    auto hasRenders() -> bool;

    /** @brief Get the number of Renders */
    auto numberOfRenders() -> size_t;

    /** @brief Get the list of Render IDs */
    [[nodiscard]] auto renderIDs() const -> std::vector<Render::Identifier>;

    /** @brief Get the list of Render names */
    [[nodiscard]] auto renderNames() const -> std::vector<std::string>;

    /**
     * @brief Creates a new Render.
     *
     * Populates the .volpkg file with a new Render directory and adds the ID to
     * the internal list of Renders.
     */
    auto newRender(std::string name = "") -> Render::Pointer;

    /** @brief Get a Render by uuid */
    [[nodiscard]] auto render(const Render::Identifier& id) const
        -> const Render::Pointer;

    /** @copydoc VolumePkg::render(const Render::Identifier&) const */
    auto render(const Render::Identifier& id) -> Render::Pointer;
    /**@}*/

    /** @name Transform Data */
    /** @brief Add a transform to the VolPkg */
    auto addTransform(const Transform3D::Pointer& transform)
        -> Transform3D::Identifier;

    /** @brief Replace an existing transform */
    void setTransform(
        const Transform3D::Identifier& id,
        const Transform3D::Pointer& transform);

    /** @brief Get a transform by ID */
    auto transform(const Transform3D::Identifier& id) -> Transform3D::Pointer;

    /** @brief Get a transform from a source volume to a target volume */
    auto transform(const Volume::Identifier& src, const Volume::Identifier& tgt)
        -> std::vector<Transform3D::Pointer>;

    /** @brief Get the list of transform IDs */
    [[nodiscard]] auto transformsIDs() const
        -> std::vector<Transform3D::Identifier>;
    /**@}*/

private:
    /** VolumePkg metadata */
    Metadata config_;
    /** The root directory of the VolumePkg */
    filesystem::path rootDir_;
    /** The subdirectory containing Volume data */
    [[nodiscard]] auto vols_dir_() const -> filesystem::path;
    /** The subdirectory containing Segmentation data */
    [[nodiscard]] auto segs_dir_() const -> filesystem::path;
    /** The subdirectory containing Render data */
    [[nodiscard]] auto rend_dir_() const -> filesystem::path;
    /** The subdirectory containing Transform data */
    [[nodiscard]] auto tfm_dir_() const -> filesystem::path;
    /** Get a list of required subdirectories */
    inline auto required_dirs_() -> std::vector<filesystem::path>;
    /** The list of all Volumes in the VolumePkg. */
    std::map<Volume::Identifier, Volume::Pointer> volumes_;
    /** The list of all Segmentations in the VolumePkg. */
    std::map<Segmentation::Identifier, Segmentation::Pointer> segmentations_;
    /** The list of all Renders in the VolumePkg. */
    std::map<Render::Identifier, Render::Pointer> renders_;
    /** The list of Transforms in the VolumePkg */
    std::map<Transform3D::Identifier, Transform3D::Pointer> transforms_;

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
    static auto InitConfig(const Dictionary& dict, int version) -> Metadata;
};
}  // namespace volcart
