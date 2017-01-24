#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>

#include "core/vc_defines.hpp"

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
    OrderedResampling() : inWidth_{0}, inHeight_{0} {}

    /**
     * @param mesh Mesh to be resampled
     * @param inWidth of the ordering matrix
     * @param inHeight Height of the ordering matrix
     */
    OrderedResampling(ITKMesh::Pointer mesh, int inWidth, int inHeight)
        : input_{mesh}, inWidth_{inWidth}, inHeight_{inHeight}
    {
    }
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
     * @param inWidth Width of the ordering matrix
     * @param inHeight Height of the ordering matrix
     */
    void setMesh(const ITKMesh::Pointer& mesh, int inWidth, int inHeight);

    /**
     * @brief Get the resampled mesh.
     */
    ITKMesh::Pointer getOutputMesh() const;

    /**
     * @brief Get the width of the resampled mesh.
     */
    int getOutputWidth() const { return outWidth_; }

    /**
     * @brief Get the height of the resampled mesh.
     */
    int getOutputHeight() const { return outHeight_; }
    //@}

    /** @name Processing */
    /**
     * @brief Compute resampled mesh.
     */
    void compute();
    //@}

private:
    ITKMesh::Pointer input_;
    ITKMesh::Pointer output_;

    /** The number of columns in the input ordering matrix */
    int inWidth_;  // how many rows
    /** The number of rows in the input ordering matrix */
    int inHeight_;  // how many points per row
    /** The number of columns in the output ordering matrix */
    int outWidth_;
    /** The number of rows in the output ordering matrix */
    int outHeight_;

    /**
     * @brief Add a face to the output mesh.
     *
     * @param a ID for the first vertex in the face
     * @param b ID for the second vertex in the face
     * @param c ID for the third vertex in the face
     */
    void add_cell_(uint32_t a, uint32_t b, uint32_t c);
};
}
}
