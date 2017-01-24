#pragma once

#include <opencv2/core.hpp>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkSmartPointer.h>

#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/core/types/PointSet.hpp"
#include "vc/core/vc_defines.hpp"

namespace volcart
{
namespace shapes
{
/**
 * @author Seth Parker
 * @date 9/18/15
 *
 * @brief Base class for shape generators
 *
 * This class provides a common interface for interacting with the various
 * shape generators (Cube, Plane, etc.) provided by the Core library. Shape
 * generators are meant to be used for unit and regression testing.
 *
 * This class internally keeps a simple list of vertices and triangular faces
 * and provides adaptor functions to convert these lists into higher level
 * data structures (ITK/VTK meshes, PointSets, etc.). A new ShapePrimitive
 * object will have no vertices or faces.
 *
 * @ingroup Shapes
 */
class ShapePrimitive
{
public:
    /**@{*/
    /** @brief Return the shape as an ITKMesh */
    ITKMesh::Pointer itkMesh();

    /** @brief Return the shape as a vtkPolyData*/
    vtkSmartPointer<vtkPolyData> vtkMesh();
    /**@}*/

    /**@{*/
    /** @brief Return the vertices as a PointSet
     *
     * Element order is: {x, y, z}
     */
    PointSet<cv::Vec3d> unorderedPoints();

    /** @brief Return the vertices and vertex normals as a PointSet
     *
     * Element order is: {x, y, z, nx, ny, nz}
     */
    PointSet<cv::Vec6d> unorderedPointNormal();
    /**@}*/

    /**@{*/
    /** @brief Return the vertices as a list of volcart::Vertex */
    std::vector<Vertex> getPoints() const { return points_; }

    /** @brief Return the faces as a list of volcart::Cell */
    std::vector<Cell> getCells() const { return cells_; }
    /**@}*/

    /**@{*/
    /** @brief Return the vertices as an OrderedPointSet
     *
     * Throws a std::runtime_error if shape does not generate ordered points
     *
     * Element order is: {x, y, z}
     */
    OrderedPointSet<cv::Vec3d> orderedPoints();

    /** @brief Return the vertices and vertex normals as an OrderedPointSet
     *
     * Throws a std::runtime_error if shape does not generate ordered vertices
     *
     * Element order is: {x, y, z, nx, ny, nz}
     */
    OrderedPointSet<cv::Vec6d> orderedPointNormal();

    /** @brief Return if the shape generator created ordered vertices */
    bool isOrdered() const { return ordered_; }

    /** @brief Return the width of the ordered vertex set */
    uint32_t orderedWidth() const { return orderedWidth_; }

    /** @brief Return the height of the ordered vertex set */
    uint32_t orderedHeight() const { return orderedHeight_; }
    /**@}*/

protected:
    /** Default constructor */
    ShapePrimitive();

    /** Vertex storage */
    std::vector<Vertex> points_;
    /** Face storage */
    std::vector<Cell> cells_;

    /** @brief Add a new vertex to the shape */
    void addVertex_(double x, double y, double z);

    /** @brief Add a new triangular face to the mesh from vertex IDs
     *
     * Automatically calls updateNormal_ as needed
     */
    void addCell_(int v1, int v2, int v3);

    /** @brief Update the normal of a vertex with a new normal component
     *
     * The stored vertex normal is an average of the normals of a vertex's
     * adjacent faces. This function reaverages the normal with the vector
     * defined by {nx, ny, nz}.
     */
    void updateNormal_(int vertID, double nx, double ny, double nz);

    /**@{*/
    /** Is the generated vertex set ordered? */
    bool ordered_;
    /** Width of the ordered vertex set */
    uint32_t orderedWidth_;
    /** Height of the ordered vertex set */
    uint32_t orderedHeight_;
    /**@}*/
};
}
}
