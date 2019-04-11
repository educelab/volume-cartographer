#pragma once

#include "vc/core/landmarks/VolumeLandmark.hpp"

namespace volcart
{
namespace landmarks
{
/**
 * @brief Boundless plane volume landmark
 *
 * Useful for specifying a ground plane or "front cover" position.
 *
 * @ingroup Landmarks
 */
class PlaneLandmark : public VolumeLandmark
{
public:
    /** Pointer type */
    using Pointer = std::shared_ptr<PlaneLandmark>;

    /** @brief Constructor with uuid and name */
    PlaneLandmark(const Identifier& uuid, const std::string& name);

    /** @brief Constructor with uuid, name, center, and normal */
    PlaneLandmark(
        const Identifier& uuid,
        const std::string& name,
        const Point& center,
        const cv::Vec3d& normal);

    /** @copydoc PlaneLandmark(const Identifier&, const std::string&) */
    static Pointer New(const Identifier& uuid, const std::string& name);

    /** @copydoc PlaneLandmark(const Identifier&, const std::string&, const
     * Point&, const cv::Vec3d&) */
    static Pointer New(
        const Identifier& uuid,
        const std::string& name,
        const Point& center,
        const cv::Vec3d& normal);

    /** @brief Set center of plane */
    void setCenter(double x, double y, double z);

    /** @overload */
    void setCenter(const Point& values);

    /** @brief Set plane normal */
    void setNormal(double x, double y, double z);

    /** @overload */
    void setNormal(const Point& values);

    /** @brief Get center of plane */
    Point getCenter() const;

    /** @brief Get plane normal */
    Point getNormal() const;

private:
    /** Plane center point */
    Point center_;

    /** Plane normal */
    cv::Vec3d normal_;

    /** Update the metadata structure */
    void update_meta_();
};
}  // namespace landmarks
}  // namespace volcart