#pragma once

#include <iostream>

// vcglib has order-specific includes. Don't let clang-format mess with them.
// clang-format off
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>
// clang-format on

#include "vc/core/types/ITKMesh.hpp"

namespace volcart
{
namespace meshing
{
/**
 * @class QuadricEdgeCollapseDecimation
 * @author Hannah Hatch
 * @date 8/12/16
 *
 * @brief Resample an ITKMesh using Quadric Edge Collapse.
 *
 * Reduce the number of vertices and faces in the mesh. Iteratively uses the
 * quadric error metric to collapse edges until the desired number of faces has
 * been reached.
 *
 * @ingroup Meshing
 */
class QuadricEdgeCollapseDecimation
{

    class VcgVertex;
    class VcgEdge;
    class VcgFace;

    /**
     * @struct VcgUsedTypes
     * @brief Associate classes with vcg types
     */
    struct VcgUsedTypes : public vcg::UsedTypes<
                              vcg::Use<VcgVertex>::AsVertexType,
                              vcg::Use<VcgEdge>::AsEdgeType,
                              vcg::Use<VcgFace>::AsFaceType> {
    };

    /**
     * @class VcgVertex
     * @brief Vertex type for a VcgMesh.
     *
     * Defines a vertex with 3D coordinates, 3D normals, face adjacency
     * information, a vertex quality measure (this stores the quadric error),
     * and a deletion marker.
     *
     * @warning vcg's QECD algorithm does not remove vertices from
     * the mesh representation. It only marks them for deletion.
     */
    class VcgVertex : public vcg::Vertex<
                          VcgUsedTypes,
                          vcg::vertex::VFAdj,
                          vcg::vertex::Coord3d,
                          vcg::vertex::Normal3d,
                          vcg::vertex::Mark,
                          vcg::vertex::Qualityd,
                          vcg::vertex::BitFlags>
    {
    public:
        vcg::math::Quadric<double>& Qd() { return q_; }

    private:
        vcg::math::Quadric<double> q_;
    };

    /**
     * @class VcgFace
     * @brief Face type for a VcgMesh.
     *
     * Defines a face with vertex adjacency information, a list of the vertices
     * that compose the face, and a deletion marker.
     *
     * @warning vcg's QECD algorithm does not remove faces from
     * the mesh representation. It only marks them for deletion.
     */
    class VcgFace : public vcg::Face<
                        VcgUsedTypes,
                        vcg::face::VFAdj,
                        vcg::face::VertexRef,
                        vcg::face::BitFlags>
    {
    };

    /**
     * @class VcgEdge
     * @brief Edge type for a VcgMesh.
     */
    class VcgEdge : public vcg::Edge<VcgUsedTypes>
    {
    };

    /**
     * @class VcgMesh
     *
     * Defines a triangular mesh composed of vertices with type VcgVertex and
     * faces with type VcgFace.
     */
    class VcgMesh
        : public vcg::tri::TriMesh<std::vector<VcgVertex>, std::vector<VcgFace>>
    {
    };

    /**
     * @typedef VertexPair
     *
     * Used to represent pairs of vertices being considered for removal by QECD.
     */
    using VertexPair = vcg::tri::BasicVertexPair<VcgVertex>;

    /**
     * @class VcgTriEdgeCollapse
     * @brief Quadric Edge Collapse Decimation.
     *
     * Templates vcg's Quadric Edge Collapse Decimation for the VcgMesh type.
     * Includes QInfoStandard which is used to retrieve information about the
     * vertices.
     */
    class VcgTriEdgeCollapse : public vcg::tri::TriEdgeCollapseQuadric<
                                   VcgMesh,
                                   VertexPair,
                                   VcgTriEdgeCollapse,
                                   vcg::tri::QInfoStandard<VcgVertex>>
    {
    public:
        using TECQ = vcg::tri::TriEdgeCollapseQuadric<
            VcgMesh,
            VertexPair,
            VcgTriEdgeCollapse,
            vcg::tri::QInfoStandard<VcgVertex>>;
        using EdgeType = VcgMesh::VertexType::EdgeType;

        VcgTriEdgeCollapse(
            const VertexPair& p, int i, vcg::BaseParameterClass* pp)
            : TECQ(p, i, pp)
        {
        }
    };

public:
    /** @name Constructors */
    //@{
    /**
     * Default constructor.
     */
    QuadricEdgeCollapseDecimation() : itkInput_{nullptr} { setDefaultParams(); }

    /**
     * @param mesh Mesh which will be decimated
     */
    explicit QuadricEdgeCollapseDecimation(const ITKMesh::Pointer& mesh)
        : itkInput_{mesh}
    {
        setDefaultParams();
    }
    //@}

    /** @name Parameters */
    //@{
    /**
     * @brief Set the input mesh.
     * @param mesh Mesh which will be decimated
     */
    void setMesh(const ITKMesh::Pointer& mesh) { itkInput_ = mesh; }

    /**
     * @brief Reset all parameters to their default values.
     *
     * All of the defaults are set by vcg with the exception of
     * PreserveBoundary and PreserveTopology.
     */
    void setDefaultParams();

    /**
     * @brief Set all parameters.
     *
     * Useful to update all of the parameters at once.
     *
     * @param newParams Set of new QECD parameters.
     */
    void setAllParams(vcg::tri::TriEdgeCollapseQuadricParameter newParams)
    {
        collapseParams_ = newParams;
    }

    /**
     * @brief Set the desired number of output faces.
     *
     * If the desired number of faces has not been set, the algorithm runs
     * until the quadric error is minimized.
     */
    void setDesiredFaces(size_t n) { desiredFaces_ = n; }

    /**
     * @brief Set the boundary weight factor.
     *
     * Default: 0.5.
     */
    void setBoundaryWeight(double weight)
    {
        collapseParams_.CosineThr = weight;
    }

    /**
     * @brief Set the cosine threshold.
     *
     * Default: \f$ \frac{\pi}{2} \f$.
     */
    void setCosineThr(double thr) { collapseParams_.CosineThr = thr; }

    /**
     * @brief Set use Fast Boundary Preserve.
     *
     * Default: False
     */
    void setFastPreserveBoundary(bool set)
    {
        collapseParams_.FastPreserveBoundary = set;
    }

    /**
     * @brief Set use vertex normal check.
     *
     * Vertex normal check includes vertex normals as part of the quadric
     * error calculation.
     *
     * Default: False.
     */
    void setNormalCheck(bool set) { collapseParams_.NormalCheck = set; }

    /**
     * @brief Set vertex normal angle threshold.
     *
     * Default: \f$ \frac{\pi}{2} \f$.
     */
    void setNormalThrRad(double rad) { collapseParams_.NormalThrRad = rad; }

    /**
     * @brief Set use optimal vertex placement.
     *
     * If enabled, for each edge collapse, a new vertex is produced which
     * minimizes the resulting quadric error. Otherwise, an existing vertex is
     * used.
     *
     * Default: True
     */
    void setOptimalPlacement(bool set)
    {
        collapseParams_.OptimalPlacement = set;
    }

    /**
     * @brief Set preserve topology.
     *
     * Default: True
     */
    void setPreserveTopology(bool set)
    {
        collapseParams_.PreserveTopology = set;
    }

    /**
     * @brief Set preserve mesh boundary.
     *
     * If set, QECD will attempt to preserve boundary features of the mesh.
     *
     * Default: True
     */
    void setPreserveBoundary(bool set)
    {
        collapseParams_.PreserveBoundary = set;
    }

    /**
     * @brief Set the quadric error threshold.
     *
     * Default: 1e-15
     */
    void setQuadricEpsilon(double epsilon)
    {
        collapseParams_.QuadricEpsilon = epsilon;
    }

    /**
     * @brief Set use mesh quality check.
     *
     * Default: True
     */
    void setQualityCheck(bool set) { collapseParams_.QualityCheck = set; }

    /**
     * @brief Set use quadric error for quality check.
     *
     * Default: False
     */
    void setQualityQuadric(bool set) { collapseParams_.QualityQuadric = set; }

    /**
     * @brief Set the quality threshold.
     *
     * Default: 0.3
     */
    void setQualityThr(double thr) { collapseParams_.QualityThr = thr; }

    /**
     * @brief Set use weighted quality measure.
     *
     * Default: False
     */
    void setQualityWeight(bool set) { collapseParams_.QualityWeight = set; }

    /**
     * @brief Set the quality weight factor.
     *
     * Has no effect if QualityWeight is not enabled.
     *
     * Default: 100
     */
    void setQualityWeightFactor(bool factor)
    {
        collapseParams_.QualityWeight = factor;
    }

    /**
     * @brief Set mesh scale factor.
     *
     * Default: 1.0
     */
    void setScaleFactor(double scale) { collapseParams_.ScaleFactor = scale; }

    /**
     * @brief Set compute quadric independent of mesh scale.
     *
     * Default: True
     */
    void setScaleIndependent(bool set)
    {
        collapseParams_.ScaleIndependent = set;
    }

    /**
     * @brief Set use area to compute quadric.
     *
     * If disabled, edge vectors are normalized for quadric computation.
     *
     * Default: True
     */
    void setUseArea(bool set) { collapseParams_.UseArea = set; }

    /**
     * @brief Set use vertex weighting for quadric computation.
     *
     * Default: False
     */
    void setUseVertexWeight(bool set) { collapseParams_.UseVertexWeight = set; }
    //@}

    /**
     * @brief Compute decimated mesh.
     *
     * If the desired number of faces has not been set, the algorithm runs
     * until the quadric error is minimized.
     */
    void compute();

    /**
     * @brief Compute decimated mesh with target number of faces.
     *
     * This computes the edge collapse decimation, targeting the desired number
     * of faces.
     */
    void compute(size_t desiredFaces);

    /**
     * @brief Get the decimated output mesh.
     */
    ITKMesh::Pointer getMesh();

private:
    /**
     * @brief Convert the ITK mesh to a vcgMesh.
     */
    void convert_mesh_to_vcg_();

    ITKMesh::Pointer itkInput_;
    VcgMesh vcgInput_;
    ITKMesh::Pointer outputMesh_;

    /** Desired number of faces in the output mesh */
    size_t desiredFaces_;
    /** QECD parameter set */
    vcg::tri::TriEdgeCollapseQuadricParameter collapseParams_;

};  // QuadricEdgeCollapse
}  // meshing
}  // volcart
