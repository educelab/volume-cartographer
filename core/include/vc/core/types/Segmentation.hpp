#pragma once

/** @file */

#include "vc/core/filesystem.hpp"
#include "vc/core/types/Annotation.hpp"
#include "vc/core/types/DiskBasedObjectBaseClass.hpp"
#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart
{

/**
 * @class Segmentation
 * @author Seth Parker
 *
 * @brief Segmentation data
 *
 * Provides access to Segmentation information stored on disk, usually inside of
 * a VolumePkg.
 *
 * A Segmentation is generated within the coordinate frame of a Volume. Use the
 * `[has\|get\|set]VolumeID()` methods to retrieve the ID of the Volume with
 * which the Segmentation is associated.
 *
 * @ingroup Types
 */
class Segmentation : public DiskBasedObjectBaseClass
{
public:
    /** Point set type */
    using PointSet = OrderedPointSet<cv::Vec3d>;

    /** Shared pointer type */
    using Pointer = std::shared_ptr<Segmentation>;

    /** @brief Load a Segmentation from file */
    explicit Segmentation(filesystem::path path);

    /** @brief Make a new Segmentation in a directory */
    Segmentation(filesystem::path path, Identifier uuid, std::string name);

    /** @copydoc Segmentation(volcart::filesystem::path path) */
    static auto New(const filesystem::path& path) -> Pointer;

    /** @copydoc Segmentation(volcart::filesystem::path path, Identifier uuid,
     * std::string name) */
    static auto New(
        const filesystem::path& path,
        const Identifier& uuid,
        const std::string& name) -> Pointer;

    /**
     * @brief Return if this Segmentation has an associated PointSet file
     *
     * Returns false if the metadata file has no `vcps` entry, the `vcps` entry
     * is `null`, or the `vcps` entry is an empty string.
     */
    [[nodiscard]] auto hasPointSet() const -> bool;

    /**
     * @brief Save a PointSet to the Segmentation file
     *
     * @warning This will overwrite the PointSet file associated with this
     * Segmentation.
     */
    void setPointSet(const PointSet& ps);

    /**
     * @brief Load the associated PointSet from the Segmentation file
     *
     * PointSet data is never cached in memory and is always loaded from disk.
     */
    [[nodiscard]] auto getPointSet() const -> PointSet;

    /**
     * @brief Return if this Segmentation has an associated AnnotationSet file
     *
     * Returns false if the metadata file has no `vcano` entry, the `vcano`
     * entry is `null`, or the `vcps` entry is an empty string.
     */
    [[nodiscard]] auto hasAnnotations() const -> bool;

    /**
     * @brief Save AnnotationSet to the Segmentation file
     *
     * @warning This will overwrite the AnnotationSet file associated with this
     * Segmentation.
     */
    void setAnnotationSet(const AnnotationSet& as);

    /**
     * @brief Load the associated AnnotationSet from the Segmentation file
     *
     * AnnotationSet data is never cached in memory and is always loaded from
     * disk.
     */
    [[nodiscard]] auto getAnnotationSet() const -> AnnotationSet;

    /**
     * @brief Return whether this Segmentation is associated with a Volume
     *
     * Returns false if the metadata file has no `volume` entry, the
     * `volume` entry is `null`, or the `volume` entry is an empty string.
     */
    [[nodiscard]] auto hasVolumeID() const -> bool;

    /** @brief Get the ID of the Volume associated with this Segmentation */
    [[nodiscard]] auto getVolumeID() const -> Volume::Identifier;

    /** @brief Set the ID of the Volume associated with this Segmentation */
    void setVolumeID(const Volume::Identifier& id);
};
}  // namespace volcart
