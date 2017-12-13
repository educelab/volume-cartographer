#pragma once

#include <boost/filesystem.hpp>

#include "vc/core/types/DiskBasedObjectBaseClass.hpp"
#include "vc/core/types/OrderedPointSet.hpp"

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
    explicit Segmentation(boost::filesystem::path path);

    /** @brief Make a new Segmentation in a directory */
    Segmentation(
        boost::filesystem::path path, Identifier uuid, std::string name);

    /** @copydoc Segmentation(boost::filesystem::path path) */
    static Pointer New(boost::filesystem::path path);

    /** @copydoc Segmentation(boost::filesystem::path path, Identifier uuid,
     * std::string name) */
    static Pointer New(
        boost::filesystem::path path, Identifier uuid, std::string name);

    /** @brief Return if this Segmentation has an associated PointSet file */
    bool hasPointSet() const
    {
        return !metadata_.get<std::string>("vcps").empty();
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
};
}
