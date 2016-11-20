#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>

#include "core/vc_defines.h"

namespace volcart
{
namespace meshing
{
/**
 * @class OrderedResampling
 * @author Hannah Hatch
 * @date 7/25/16
 *
 * @brief Resample an ITKMesh using point ordering information.
 *
 * Reduce the number of points and faces in a mesh by removing every other point
 * along the horizontal and vertical axes of the ordering matrix defined by the
 * width and height parameters. Assumes the input mesh was constructed from an
 * OrderedPointSet using OrderedPointSetMesher.
 *
 * @warning This function assumes the input mesh was constructed from an
 * OrderedPointSet using OrderedPointSetMesher, and will produce undesired
 * results for any other type of triangulation.
 *
 * @ingroup Meshing
 *
 * @see examples/src/OrderedResamplingExample.cpp
 *      test/OrderedResamplingTest.cpp
 */
class OrderedResampling
{
public:
    /** @name Constructors */
    //@{
    OrderedResampling();

    /**
     * @param mesh Mesh to be resampled
     * @param in_width Width of the ordering matrix
     * @param in_height Height of the ordering matrix
     */
    OrderedResampling(ITKMesh::Pointer mesh, int in_width, int in_height);
    //@}

    /** @name Input/Output */
    //@{
    /**
     * @brief Set the input mesh and the dimensions of the ordering matrix.
     *
     * Width defines the number of vertices in each row of the ordering matrix.
     * Height defines the number of rows in the ordering matrix.
     *
     * @param mesh Mesh to be resampled
     * @param in_width Width of the ordering matrix
     * @param in_height Height of the ordering matrix
     */
    void setMesh(ITKMesh::Pointer mesh, int in_width, int in_height);

    /**
     * @brief Get the resampled mesh.
     */
    ITKMesh::Pointer getOutputMesh() const;

    /**
     * @brief Get the width of the resampled mesh.
     */
    int getOutputWidth() const;

    /**
     * @brief Get the height of the resampled mesh.
     */
    int getOutputHeight() const;
    //@}

    /** @name Processing */
    /**
     * @brief Compute resampled mesh.
     */
    void compute();
    //@}

private:
    ITKMesh::Pointer _input;
    ITKMesh::Pointer _output;

    /** The number of columns in the input ordering matrix */
    int _inWidth;  // how many rows
    /** The number of rows in the input ordering matrix */
    int _inHeight;  // how many points per row
    /** The number of columns in the output ordering matrix */
    int _outWidth;
    /** The number of rows in the output ordering matrix */
    int _outHeight;

    /**
     * @brief Add a face to the output mesh.
     *
     * @param a ID for the first vertex in the face
     * @param b ID for the second vertex in the face
     * @param c ID for the third vertex in the face
     */
    void _addCell(unsigned long a, unsigned long b, unsigned long c);
};  // OrderedResampling
}  // meshing
}  // volcart
