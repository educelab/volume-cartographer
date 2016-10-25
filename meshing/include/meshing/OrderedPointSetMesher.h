
/**
 * @class OrderedPointSetMesher
 * @author Hannah Hatch
 * @date 8/23/16
 *
 * @brief Creates a mesh using an OrderedPointSet
 *
 * This class takes in an OrderedPointSet and creates
 * a mesh of ordered points and triangular faces.
 * It then returns a pointer to this mesh.
 *
 * @ingroup Meshing
 *
 * @see common/types/OrderedPointSet.h
 *      examples/src/OrderedPointSetMesherExample.cpp
 *      meshing/test/OrderedPointSetMesherTest.cpp
 */

#pragma once

#include "core/types/OrderedPointSet.h"
#include "core/types/Point.h"
#include "core/vc_defines.h"

namespace volcart
{
namespace meshing
{
class OrderedPointSetMesher
{
public:
    /** @name Initializers */
    //@{
    /**
     * Creates a member of the class without setting the original PointSet
     */
    OrderedPointSetMesher();

    /**
     * Creates a member of the class and sets the original PointSet
     * @param points OrderedPointSet that you want to use to create the mesh
     */
    OrderedPointSetMesher(OrderedPointSet<Point3d> points) : input_(points) {}
    //@}

    /** @name Input/Output */
    //@{
    /**
     * Sets the initial OrderedPointSet
     * @param points OrderedPointSet that you want to use to create the mesh
     */
    void setPointSet(OrderedPointSet<Point3d> points) { input_ = points; }

    /**
     * Returns a pointer to the mesh that was generated
     * @return Pointer to an ITK Mesh
     */
    ITKMesh::Pointer getOutputMesh() const { return output_; }
    //@}

    /**
     * @brief Function that creates the mesh
     *
     * This function takes the points in the OrderedPointSet and adds them
     * to the mesh. It then adds the cells by taking 4 points that make
     * a square and then creating two faces by "cutting it in half".
     */
    void compute();

private:
    /** OrderedPointSet that contains the points for the mesh */
    OrderedPointSet<Point3d> input_;

    /** Pointer to the mesh that is generated */
    ITKMesh::Pointer output_;

    /**
     * @brief Used to add a cell to the ITK Mesh
     *
     * This function adds a cell to the mesh and
     * assumes that the corners of the cell are the
     * points given
     *
     * @param a ITK Point that makes up one corner of the cell
     * @param b ITK Point that makes up one corner of the cell
     * @param c ITK Point that makes up one corner of the cell
     */
    void addCell_(size_t a, size_t b, size_t c);
};

}  // meshing
}  // volcart
