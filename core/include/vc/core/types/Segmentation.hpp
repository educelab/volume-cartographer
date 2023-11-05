#pragma once

/** @file */

#include "vc/core/filesystem.hpp"
#include "vc/core/types/DiskBasedObjectBaseClass.hpp"
#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/Volume.hpp"

#include <variant>

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

    /** Annotation type */
    using Annotation = cv::Vec<std::variant<long, double>, 3>;

    /** Annotation type (raw = only doubles) */
    using AnnotationRaw = cv::Vec3d;

    /** Annotation set type */
    using AnnotationSet = OrderedPointSet<Annotation>;

    /** Annotation set type (raw = only doubles) */
    using AnnotationSetRaw = OrderedPointSet<AnnotationRaw>;

    /** Shared pointer type */
    using Pointer = std::shared_ptr<Segmentation>;

    /** @brief Load a Segmentation from file */
    explicit Segmentation(filesystem::path path);

    /** @brief Make a new Segmentation in a directory */
    Segmentation(filesystem::path path, Identifier uuid, std::string name);

    /** @copydoc Segmentation(volcart::filesystem::path path) */
    static Pointer New(filesystem::path path);

    /** @copydoc Segmentation(volcart::filesystem::path path, Identifier uuid,
     * std::string name) */
    static Pointer New(
        filesystem::path path, Identifier uuid, std::string name);

    /** @brief Return if this Segmentation has an associated PointSet file */
    bool hasPointSet() const
    {
        return metadata_.hasKey("vcps") && !metadata_.get<std::string>("vcps").empty();
    }

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
    PointSet getPointSet() const;

    /** @brief Return if this Segmentation has an associated AnnotationSet file */
    bool hasAnnotations() const
    {
        return metadata_.hasKey("vcano") && !metadata_.get<std::string>("vcano").empty();
    }

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
     * AnnotationSet data is never cached in memory and is always loaded from disk.
     */
    AnnotationSet getAnnotationSet() const;

    /** @brief Return whether this Segmentation is associated with a Volume */
    bool hasVolumeID() const
    {
        return metadata_.hasKey("volume") && !getVolumeID().empty();
    }

    /** @brief Get the ID of the Volume associated with this Segmentation */
    Volume::Identifier getVolumeID() const
    {
        return metadata_.get<Volume::Identifier>("volume");
    }

    /** @brief Set the ID of the Volume associated with this Segmentation */
    void setVolumeID(const Volume::Identifier& id)
    {
        metadata_.set<std::string>("volume", id);
        metadata_.save();
    }
};
}  // namespace volcart
