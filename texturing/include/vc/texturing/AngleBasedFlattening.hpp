/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contributor(s):
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#pragma once

#include <exception>
#include <iostream>
#include <memory>

#include <itkQuadEdgeMeshBoundaryEdgesMeshFunction.h>
#include <opencv2/core.hpp>

#include "vc/core/types/HalfEdgeMesh.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart
{
namespace texturing
{
/**
 * @class AngleBasedFlattening
 * @author Seth Parker
 * @date 6/9/16
 *
 * @copyright This implementation is ported from the implementation in Blender,
 * and as such is subject to the terms of the GPL v2.
 *
 * @brief Computes a 2D parameterization of a triangular mesh using
 * Angle-Based Flattening
 *
 * This class attempts to find the ideal angles that minimize the angular
 * distortion of the parameterized mesh. These idealized angles are then fed
 * into a least-squares conformal maps algorithm which solves for the actual
 * parameterized UV positions.
 *
 * Based on the paper by Sheffer and de Sturler
 * @cite sheffer2001parameterization.
 *
 * @ingroup UV
 */
class AngleBasedFlattening
{
public:
    /**@{*/
    /** @brief Default constructor */
    AngleBasedFlattening()
        : useABF_{true}, maxABFIterations_{DEFAULT_MAX_ABF_ITERATIONS}
    {
    }

    /** @brief Construct and set the input mesh */
    explicit AngleBasedFlattening(ITKMesh::Pointer mesh)
        : useABF_{true}
        , maxABFIterations_{DEFAULT_MAX_ABF_ITERATIONS}
        , mesh_{mesh}
    {
    }
    /**@}*/

    /**@{*/
    /** @brief Set the input mesh */
    void setMesh(const ITKMesh::Pointer& mesh) { mesh_ = mesh; }

    /** @brief Get the flattened surface as a mesh */
    ITKMesh::Pointer getMesh();

    /** @brief Get the flattened surface as a UV map */
    volcart::UVMap getUVMap();
    /**@}*/

    /**@{*/
    /**
     * @brief Set whether to perform Angle-based Flattening computation
     *
     * Setting this value to `false` results in a mesh that is flattened using
     * only the LSCM algorithm.
     */
    void setUseABF(bool a) { useABF_ = a; }

    /** @brief Set the max number of ABF minimization iterations
     *
     * ABF will often compute the ideal parameterization angles in fewer than
     * 5 iterations. This implementation has a bug in how it calculates the
     * minimization metric (I think?), and this can occasionally be too many/few
     * iterations for a particular mesh. Adjusting this value might help.
     */
    void setABFMaxIterations(int i) { maxABFIterations_ = i; }
    /**@}*/

    /**@{*/
    /** @brief Compute the parameterization */
    UVMap compute();
    /**@}*/

private:
    /**@{*/
    /** Convert the input mesh to a HalfEdgeMesh */
    void fill_half_edge_mesh_();

    /** Compute the ABF minimized angles */
    void solve_abf_();

    /** Scale the angles to avoid degenerate cases */
    void scale_();

    /** Precalculate the sin and cos for each angle in the HalfEdgeMesh */
    void compute_sines_();

    /** Calculate the magnitude of the error vector */
    double compute_gradient_();

    /** Calculate the magnitude of the error vector for a specific angle */
    double compute_gradient_alpha_(const HalfEdgeMesh::EdgePtr& e0)
    {
        return std::pow(e0->angle->alpha - e0->angle->phi, 2) *
               e0->angle->weight;
    }

    /** Compute the edge length constraint */
    double compute_sin_product_(const HalfEdgeMesh::VertPtr& v);

    /** @copydoc compute_sin_product_() */
    double compute_sin_product_(
        const HalfEdgeMesh::VertPtr& v, HalfEdgeMesh::IDType aId);

    /** Fill the solver matric and perform the inversion calculation */
    bool invert_matrix_();

    /** Whether to use ABF minimization */
    bool useABF_;
    /** Maximum number of ABF minimization iterations */
    int maxABFIterations_;
    /** Threshold for determining a properly minimized mesh */
    double limit_;
    /**@}*/

    /**@{*/
    /** Compute the parameterization using LSCM */
    void solve_lscm_();

    /** Get the ID's of the vertices with the minimum and maximum positions */
    std::pair<HalfEdgeMesh::IDType, HalfEdgeMesh::IDType>
    get_min_max_point_ids_();

    /** Get the ID's of the vertices with the minimum and maximum Z-positions */
    std::pair<HalfEdgeMesh::IDType, HalfEdgeMesh::IDType>
    get_min_z_point_ids_();

    /** Generate a good starting UV position for two "pinned" points. This is
     * supposedly arbitrary. */
    void compute_pin_uv_();
    /**@}*/

    /** Input mesh */
    ITKMesh::Pointer mesh_;

    /** HalfEdgeMesh for processing */
    HalfEdgeMesh heMesh_;

    /** Interior Vertices */
    std::map<ITKMesh::PointIdentifier, ITKMesh::PointIdentifier> interior_;

    /** ABF metric for interior points */
    std::vector<double> bInterior_;

    /** Something used by invert_matrix_() */
    cv::Mat j2dt_;

    /** ID of pinned point #1 */
    HalfEdgeMesh::IDType pin0_;

    /** ID of pinned point #2 */
    HalfEdgeMesh::IDType pin1_;

    /** Shift values around a triangle */
    template <typename T>
    void shift3_(T& a, T& b, T& c)
    {
        T tmp = a;
        a = c;
        c = b;
        b = tmp;
    }

    /** Default maximum number of ABF iterations */
    static const int DEFAULT_MAX_ABF_ITERATIONS = 5;
};
}
}
