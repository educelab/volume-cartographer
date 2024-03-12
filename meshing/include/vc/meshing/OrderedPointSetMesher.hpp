#pragma once

/** @file */

#include <cstddef>

#include <opencv2/core.hpp>

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/OrderedPointSet.hpp"

namespace volcart::meshing
{
/**
 * @brief Generate an ordered mesh from an OrderedPointSet.
 *
 * Create a mesh from an OrderedPointSet, using the ordering information to
 * generate a triangulation of the vertices. Triangulation relies upon the
 * ordering information inherent to the OrderedPointSet and is independent of
 * the actual 3D position of vertices.
 *
 * Vertices are grouped into "squares" according to their position within
 * the ordering matrix. These squares are then subdivided into two
 * triangles and added to the output mesh.
 *
 * Vertex normals are computed using CalculateNormals.
 *
 * @ingroup Meshing
 *
 * @see common/types/OrderedPointSet.h
 *      examples/src/OrderedPointSetMesherExample.cpp
 *      meshing/test/OrderedPointSetMesherTest.cpp
 */
class OrderedPointSetMesher
{
public:
    /** Point set convenience alias */
    using PointSet = OrderedPointSet<cv::Vec3d>;

    /**@{*/
    /** @brief Default Constructor */
    OrderedPointSetMesher() = default;

    /** @brief Construct with OrderedPointSet to be converted */
    explicit OrderedPointSetMesher(PointSet points) : input_{std::move(points)}
    {
    }
    /**@}*/

    /**@{*/
    /** @brief Set the input OrderedPointSet */
    void setPointSet(const PointSet& points) { input_ = points; }

    /**
     * @brief Set whether to compute a triangulation from point ordering
     *
     * When enabled, use point ordering to generate a triangulation.
     * Otherwise, the resulting mesh is only filled with vertices.
     *
     * Default: Enabled
     */
    void setComputeTriangulation(bool b) { generateTriangles_ = b; }

    /** @brief Get the generated mesh */
    ITKMesh::Pointer getOutputMesh() const { return output_; }
    /**@}*/

    /**@{*/
    /**
     * @brief Compute the mesh triangulation.
     */
    ITKMesh::Pointer compute();
    /**@}*/

private:
    PointSet input_;
    ITKMesh::Pointer output_;

    bool generateTriangles_{true};

    /**
     * @brief Add a face to the output mesh.
     *
     * @param a ID for the first vertex in the face
     * @param b ID for the second vertex in the face
     * @param c ID for the third vertex in the face
     */
    void add_cell_(std::size_t a, std::size_t b, std::size_t c);
};
}  // namespace volcart::meshing
