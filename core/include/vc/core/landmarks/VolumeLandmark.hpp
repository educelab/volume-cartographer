#pragma once

/** @file */

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>

#include "vc/core/types/Metadata.hpp"

namespace volcart
{
namespace landmarks
{

/** @brief Implemented Volume Landmark types */
enum class Type { Point, Plane, Polyline };

/**
 * @brief Base class for Volume Landmark types
 *
 * @ingroup Landmarks
 */
class VolumeLandmark
{
public:
    /** Default constructor */
    VolumeLandmark() = delete;

    /** @brief Identifier type */
    using Identifier = std::string;

    /** @brief 3D Point type */
    using Point = cv::Vec3d;

    /** @brief Pointer type */
    using Pointer = std::shared_ptr<VolumeLandmark>;

    /** @brief Get the unique identifier */
    Identifier id() const;

    /** @brief Get human-readable landmark name */
    std::string name() const;

    /** @brief Get the landmark type */
    Type type() const;

    /** @brief Write a landmark metadata file to the path provided */
    static void Write(const boost::filesystem::path& path, const Pointer& ldm);

    /** @brief Read a landmark metadata file from the path provided */
    static VolumeLandmark::Pointer Read(const boost::filesystem::path& path);

protected:
    /** Constructor with uuid, name, and type */
    VolumeLandmark(const Identifier& uuid, const std::string& name, Type type);

    /** Metadata */
    Metadata metadata_;

    /** Landmark type */
    Type type_;
};
}  // namespace landmarks
}  // namespace volcart