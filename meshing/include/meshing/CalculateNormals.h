//
// Created by Hannah Hatch on 7/26/16.
//

#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

#include "core/vc_defines.h"

/**
 * @class CalculateNormals
 * @author Hannah Hatch
 * @date 7/26/16
 *
 * @brief Calculates the normals to the points for ITK meshes
 *
 * This class calculates the normals to the points and then adds them
 * to a copy of the input mesh and returns a pointer to the new mesh
 * with normals.
 *
 * @ingroup Meshing
 */

namespace volcart
{
namespace meshing
{
class CalculateNormals
{
public:
    //** @name Constructors */
    //@{
    /**
     * @brief Constructs a member of the class without setting an input mesh
     */
    CalculateNormals();

    /**
     * @brief Constructs a member of the class and sets an input mesh
     * @param mesh Input Mesh whose normals you want computed
     */
    CalculateNormals(ITKMesh::Pointer mesh);
    //@}

    //** @name Input/Output */
    //@{
    /**
     * @brief Sets the input mesh
     * @param mesh Input Mesh whose normals you want calculated
     */
    void setMesh(ITKMesh::Pointer mesh);

    /**
     * @brief Returns a pointer to the mesh with computed normals
     * @return Points to the output mesh
     */
    ITKMesh::Pointer getMesh() const;
    //@}

    /**
     * @brief Computes the normals and assigns them to the mesh
     *
     * This function calls two other functions which do the work. It calls
     * _computeNormals to actually get the normals and then assigns them with
     * _assignToMesh.
     *
     * @see _computeNormals()
     * @see _assignToMesh()
     */
    void compute();

private:
    /**
     * @brief Does the normals computation
     *
     * This function takes the cross product of the edge vectors
     * and then adds the normal of the face to the sum of the normals
     * of all the vertices.
     */
    void _computeNormals();

    /**
     * @brief Assigns the normals to the output mesh
     *
     * This function takes the vector of normals and then assigns them to each
     * of the points in the output mesh.
     */
    void _assignToMesh();

    /** Mesh whose normals need to be calculated */
    ITKMesh::Pointer _input;

    /** Mesh with normals calculated */
    ITKMesh::Pointer _output;

    /** @brief Normals that were calculated, sorted by pointID
     * The normals are stored in a vector for convience.
     * */
    std::vector<cv::Vec3d> _vertex_normals;
};
}  // meshing
}  // volcart
