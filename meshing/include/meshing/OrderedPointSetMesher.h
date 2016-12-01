#pragma once

#include "core/types/OrderedPointSet.h"
#include "core/types/Point.h"
#include "core/vc_defines.h"

namespace volcart
{
namespace meshing
{
/**
 * @class OrderedPointSetMesher
 * @author Hannah Hatch
 * @date 8/23/16
 *
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
    /** @name Constructors */
    //@{
    OrderedPointSetMesher();

    /**
     * @param points OrderedPointSet to be meshed
     */
    OrderedPointSetMesher(OrderedPointSet<Point3d> points)
        : input_(std::move(points))
    {
    }
    //@}

    /** @name Input/Output */
    //@{
    /**
     * Set the input OrderedPointSet.
     * @param points OrderedPointSet to be meshed
     */
    void setPointSet(OrderedPointSet<Point3d> points)
    {
        input_ = std::move(points);
    }

    /**
     * @brief Get the generated mesh.
     */
    ITKMesh::Pointer getOutputMesh() const { return output_; }
    //@}

    /** @name Processing */
    //@{
    /**
     * @brief Compute the mesh triangulation.
     */
    void compute();
    //@}

private:
    OrderedPointSet<Point3d> input_;
    ITKMesh::Pointer output_;

    /**
     * @brief Add a face to the output mesh.
     *
     * @param a ID for the first vertex in the face
     * @param b ID for the second vertex in the face
     * @param c ID for the third vertex in the face
     */
    void addCell_(size_t a, size_t b, size_t c);
};

}  // meshing
}  // volcart
