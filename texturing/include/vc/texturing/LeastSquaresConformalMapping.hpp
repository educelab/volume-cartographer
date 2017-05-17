#pragma once

#include <iostream>

#include <Eigen/Geometry>

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart
{
namespace texturing
{
/**
 * @class LeastSquaresConformalMapping
 * @author Seth Parker
 * @date 5/17/16
 *
 * @brief Computes a 2D parameterization of a triangular mesh using Least
 * Squares Conformal Mapping (LSCM).
 *
 * @ingroup UV
 */
class LeastSquaresConformalMapping
{
public:
    /**@{*/
    /** @brief Default constructor */
    LeastSquaresConformalMapping() = default;

    /** @brief Constructor with mesh parameter */
    explicit LeastSquaresConformalMapping(ITKMesh::Pointer input);
    /**@}*/

    /**@{*/
    /** @brief Set the input mesh */
    void setMesh(const ITKMesh::Pointer& input);

    /** @brief Get the flattened surface as a mesh */
    ITKMesh::Pointer getMesh();

    /** @brief Get the flattened surface as a UV map */
    UVMap getUVMap();
    /**@}*/

    /**@{*/
    /** @brief Compute the parameterization */
    UVMap compute();
    /**@}*/

private:
    /** Convert the mesh into eigen matrices */
    void fill_eigen_matrices_();
    /** Clear the eigen matrices */
    void empty_eigen_matrices_();
    /** Calculate the surface area of the mesh */
    double area_(const Eigen::MatrixXd& v, const Eigen::MatrixXi& f);

    /** Input mesh */
    ITKMesh::Pointer mesh_;
    /** Surface area of the input mesh */
    double startingArea_;
    /** Vertices Eigen matrix */
    Eigen::MatrixXd vertices_;
    /** Face Eigen matrix */
    Eigen::MatrixXi faces_;
    /** Vertex UV Eigen matrix */
    Eigen::MatrixXd verticesUV_;
};
}  // texturing
}  // volcart
