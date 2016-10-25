//
// Created by Hannah Hatch on 7/25/16.
/* Algorithm takes in a base mesh and reduces the number of points and faces by
 * removing every other point along
 * horizontal and vertical axes*/

/**
 * @class OrderedResampling
 * @author Hannah Hatch
 * @date 7/25/16
 *
 * @brief Algorithm that reduces the number of points and faces in a mesh
 *
 * This class takes in a base mesh and reduced the number of points
 * and faces by removing every other point along the horizatal
 * and vertical axes.
 *
 * @ingroup Meshing
 *
 * @see examples/src/OrderedResamplingExample.cpp
 *      test/OrderedResamplingTest.cpp
 */
#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>

#include "core/vc_defines.h"

namespace volcart
{
namespace meshing
{
class OrderedResampling
{
public:
    /**
     * Creates a member of the class without initalizing anything
     */
    OrderedResampling();

    /**
     * @brief Creates a member of the class and initializes everything
     *
     * The width of the height is how many rows of points there are.
     * The height of the mesh is how many points are in each row.
     *
     * @param mesh Mesh you want to resample
     * @param in_width Width of the mesh
     * @param in_height Height of the mesh
     */
    OrderedResampling(ITKMesh::Pointer mesh, int in_width, int in_height);

    /**
     * @brief Sets the input mesh and the height and width of that mesh
     *
     * The width of the height is how many rows of points there are.
     * The height of the mesh is how many points are in each row.
     *
     * @param mesh Mesh you want to resample
     * @param in_width Width of the mesh
     * @param in_height Height of the mesh
     */
    void setMesh(ITKMesh::Pointer mesh, int in_width, int in_height);

    /**
     * @brief Gets a Pointer to the resampled mesh
     * @return A pointer to an ITK mesh
     */
    ITKMesh::Pointer getOutputMesh() const;

    /**
     * @brief Gets the width of the resampled mesh
     *
     * @return Width of the resampled mesh
     */
    int getOutputWidth() const;

    /**
     * @brief Gets the height of the resampled mesh
     * @return Height of the resampled mesh
     */
    int getOutputHeight() const;

    /**
     * @brief Resamples the mesh
     *
     * This functions walks through the points of the mesh
     * and removes every other point along the horizantal
     * and vertical axes. It then recreates the faces based
     * the new set of points.
     *
     * @warning This function assumed that the mesh is ordered and will not work
     * for an unordered mesh
     */
    void compute();

private:
    /** The mesh that is to be resampled */
    ITKMesh::Pointer _input;

    /** The number of rows in the mesh */
    int _inWidth;   // how many rows

    /** The number of points per row in the mesh*/
    int _inHeight;  // how many points per row

    /** Pointer to the mesh that has been resampled*/
    ITKMesh::Pointer _output;
    /**The number of rows in the resampled mesh */
    int _outWidth;
    /**The number of points per row in the resampled */
    int _outHeight;

    /**
     * @brief Used to add a cell to an ITK Mesh
     *
     * This function adds a cell to an ITK mesh
     * based on the three points given to it.
     *
     * @param a ITK Point that makes up one corner of the cell
     * @param b ITK Point that makes up one corner of the cell
     * @param c ITK Point that makes up one corner of the cell
     */
    void _addCell(unsigned long a, unsigned long b, unsigned long c);

};  // OrderedResampling
}  // meshing
}  // volcart
