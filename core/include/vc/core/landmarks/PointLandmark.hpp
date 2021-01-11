#pragma once

/** @file */

#include "vc/core/landmarks/VolumeLandmark.hpp"

namespace volcart
{
namespace landmarks
{

/**
 * @brief 3D Point volume landmark
 *
 * Useful for specifying the location of features-of-interest.
 *
 * @ingroup Landmarks
 */
class PointLandmark : public VolumeLandmark
{
public:
    /** @brief Pointer type */
    using Pointer = std::shared_ptr<PointLandmark>;

    /** @brief Constructor with uuid and name */
    PointLandmark(const Identifier& uuid, const std::string& name);

    /** @brief Constructor with uuid, name, and position */
    PointLandmark(
        const Identifier& uuid, const std::string& name, const Point& pos);

    /** @copydoc PointLandmark(const Identifier&, const std::string&) */
    static Pointer New(const Identifier& uuid, const std::string& name);

    /** @copydoc PointLandmark(const Identifier&, const std::string&, const
     * Point&) */
    static Pointer New(
        const Identifier& uuid, const std::string& name, const Point& position);

    /** @brief Set landmark position */
    void setPosition(const Point& pos);

    /** @overload */
    void setPosition(double x, double y, double z);

    /** @brief Get landmark position */
    Point getPosition() const;

private:
    /** Landmark position */
    Point position_;

    /** Update the metadata structure */
    void update_meta_();
};
}  // namespace landmarks
}  // namespace volcart