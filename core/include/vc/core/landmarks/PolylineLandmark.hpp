#pragma once

#include "vc/core/landmarks/VolumeLandmark.hpp"

namespace volcart
{
namespace landmarks
{
/**
 * @brief Polyline (multi-segment line) volume landmark
 *
 * Useful for specifying center line of irregularly shaped object
 *
 * @ingroup Landmarks
 */
class PolylineLandmark : public VolumeLandmark
{
public:
    using Polyline = std::vector<Point>;
    /** @brief Pointer type */
    using Pointer = std::shared_ptr<PolylineLandmark>;

    /** @brief Constructor with uuid and name */
    PolylineLandmark(const Identifier& uuid, const std::string& name);

    /** @brief Constructor with uuid, name, and Polyline */
    PolylineLandmark(
        const Identifier& uuid, const std::string& name, Polyline poly);

    /** @copydoc PolylineLandmark(const Identifier&, const std::string&) */
    static Pointer New(const Identifier& uuid, const std::string& name);

    /** @copydoc PolylineLandmark(const Identifier&, const std::string&,
     * Polyline)*/
    static Pointer New(
        const Identifier& uuid, const std::string& name, const Polyline& poly);

    /** @brief Set the Polyline */
    void setPolyline(Polyline p);

    /** @brief Get the Polyline */
    Polyline getPolyline() const;

    /** @brief Add Point to the end of the Polyline */
    void addPoint(const Point& pt);

    /** @overload */
    void addPoint(double x, double y, double z);

private:
    /** Polyline */
    Polyline poly_;

    /** Update the metadata structure */
    void update_meta_();
};
}  // namespace landmarks
}  // namespace volcart