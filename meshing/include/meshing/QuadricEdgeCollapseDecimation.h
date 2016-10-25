//
// Created by Hannah Hatch on 8/12/16.
//
#pragma once

#include <iostream>
// vcglib has order-specific includes, so disable clang-format for these so they
// don't get reordered
// clang-format off
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>
// clang-format on
#include "core/vc_defines.h"

/**
 * @class QuadricEdgeCollapseDecimation
 * @author Hannah Hatch
 * @date 8/12/16
 *
 * @brief Resamples the mesh using a Quadric Edge Collapse algorithm
 *
 * This class reduces the mesh by using a quadric error metric, a matrix,
 * to decide if it should collapse and edge and recalculate the points
 * and faces. It does this repeatedly until the desired number of faces
 * is reached.
 */

namespace volcart
{
namespace meshing
{

class QuadricEdgeCollapseDecimation
{

    ///// Basic Datastructure (VCG Mesh) /////
    /** @name VCG Data structures */
    //@{
    class VcgVertex;
    class VcgEdge;
    class VcgFace;

    /**
     * @struct VcgUsedTypes
     * @brief Declaration of vcg types
     *
     * Sets up the vertex, edge, and face types
     * for the vcg mesh
     */
    struct VcgUsedTypes : public vcg::UsedTypes<
                              vcg::Use<VcgVertex>::AsVertexType,
                              vcg::Use<VcgEdge>::AsEdgeType,
                              vcg::Use<VcgFace>::AsFaceType> {
    };
    /**
     * @class VcgVertex
     * @brief Sets up the details of the vertexes for the vcgMesh
     *
     * This sets the vertex for a 3D mesh with point normals.
     *
     * It includes a place to mark if the vertex is actually supposed to
     * be part of the mesh since vcg doesn't actually delete the vertexes
     * in decimation, it simply marks them as gone.
     *
     * It also sets up a a place for the vertex to store all of the faces
     * that it is adjacent to.
     *
     * @newline
     *
     * This class is also where the quadric is set up to compute the
     * error of an edge collapse
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
        // Used to compute error as a result of an edge collapse
        vcg::math::Quadric<double>& Qd() { return q; }
    private:
        vcg::math::Quadric<double> q;
    };

    /**
     * @class VcgFace
     * @brief Sets up the details of the faces for the vcgMesh
     *
     * This sets up the faces so that they have a list of all
     * vertices they are adjacent to. The list of vertices of
     * the face are stored in VertexRef. It also stores a list
     * of flags to keep track of things such as deletion.
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
     * @brief Sets up the edges of the vcgMesh
     */
    class VcgEdge : public vcg::Edge<VcgUsedTypes>
    {
    };

    /**
     * @class VcgMesh
     * @brief Sets up a vcgMesh
     *
     * This sets up a vcgMesh with triangular faces and uses
     * the vertexes and faces that were set up previous to this.
     */
    class VcgMesh
        : public vcg::tri::TriMesh<std::vector<VcgVertex>, std::vector<VcgFace>>
    {
    };
    //@}

    ///// Edge Collapse typedefs & classes /////
    /** @name Edge Collapse classes & typedefs */
    //@{
    /**
     * @typedef VertexPair
     * @brief Defines a pair of verices of the vcgVertex type defined earlier
     */
    typedef vcg::tri::BasicVertexPair<VcgVertex> VertexPair;

    /**
     * @class VcgTriEdgeCollapse
     * @brief Used to performing the Edge Collapse computation
     *
     * This is the class that is used to prepare for, and perform the
     * edge collapse computation. It sets the meshtype to be the type defined
     * previously. It also has the QInfoStandard which is used to retrieve
     * information about the vertex type.
     *
     * @newline
     *
     * This class also defines the Edgetype for the mesh and several
     * the type TriEdgeCollapse.
     *
     * @newline
     *
     * In the definition of this class, it defines the error metric using
     * TECQ. It also takes a pair of vertices that define an edge that
     * is being considered for collapse.
     */
    class VcgTriEdgeCollapse : public vcg::tri::TriEdgeCollapseQuadric<
                                   VcgMesh,
                                   VertexPair,
                                   VcgTriEdgeCollapse,
                                   vcg::tri::QInfoStandard<VcgVertex>>
    {
    public:
        typedef vcg::tri::TriEdgeCollapseQuadric<
            VcgMesh,
            VertexPair,
            VcgTriEdgeCollapse,
            vcg::tri::QInfoStandard<VcgVertex>>
            TECQ;
        typedef VcgMesh::VertexType::EdgeType EdgeType;
        inline VcgTriEdgeCollapse(
            const VertexPair& p, int i, vcg::BaseParameterClass* pp)
            : TECQ(p, i, pp){};
    };
    //@}

public:
    // Initializers
    /** @name Initializers*/
    //@{
    /**
     * @brief Initializes a member of the class without setting the input mesh
     */
    QuadricEdgeCollapseDecimation();

    /**
     * @brief Initializes a member of the class and sets the input mesh
     * @param mesh Pointer to an ITK Mesh that needs to be decimated
     */
    QuadricEdgeCollapseDecimation(ITKMesh::Pointer mesh);
    //@}

    /** @name Parameters */
    //@{
    /**
     * @brief Sets the mesh that is to be decimated
     * @param mesh Pointer to an ITK mesh
     */
    void setMesh(ITKMesh::Pointer mesh);

    /**
     * @brief Sets all the parameters to their defaults
     * All of the defaults are set by VCG with the except
     * of PreserverBoundary and PreserveTopology
     */
    void setDefaultParams();
    /**
     * @brief Sets the VCG parameter type
     * Sets the Parameters for VCG mesh based on a set of
     * parameters that is passed in. This is useful if
     * you need to change a lot of the parameters.
     * @param newParams VCG parameter type with all parameters set
     */
    void setAllParams(vcg::tri::TriEdgeCollapseQuadricParameter newParams)
    {
        collapseParams_ = newParams;
    }

    // Defaults set by VCG, with the exception of PreserveBoundary and
    // PreserveTopology
    /**
     * @brief Sets desired number of faces
     * @param n Number of faces you want
     */
    void setDesiredFaces(size_t n) { desiredFaces_ = n; };
    /**
     * @brief Sets the boundary weight
     * The default is .5.
     * @param weight Desired Boundary weight
     */
    void setBoundaryWeight(double weight)
    {
        collapseParams_.CosineThr = weight;
    }  // Default: .5
    /**
     * @brief Sets the Cosine Angle
     * The default is pi/2.
     * @param thr Desired Angle
     */
    void setCosineThr(double thr)
    {
        collapseParams_.CosineThr = thr;
    }  // Default: cos(M_PI/2)
    /**
     * @brief Sets whether or not to use Fast Boundary Preserve
     * The defualt is false.
     * @param set bool to tell if you want to use this feature.
     */
    void setFastPreserveBoundary(bool set)
    {
        collapseParams_.FastPreserveBoundary = set;
    }  // Default: false
    /**
     * @brief Sets whether or not it checks the point normals
     * The default is false.
     * @param set Bool to tell if you want to use this feature
     */
    void setNormalCheck(bool set)
    {
        collapseParams_.NormalCheck = set;
    }  // Default: false
    /**
     * @brief Sets the angle of the normals to the plane
     * The defualt is pi/2.
     * @param rad Desired Angle to the plane
     */
    void setNormalThrRad(double rad)
    {
        collapseParams_.NormalThrRad = rad;
    }  // Default: M_PI/2
    /**
     * @brief Sets whether or not you want the optimal placement of the edges
     * The default is true.
     * @param set Bool to tell if you want to use this feature.
     */
    void setOptimalPlacement(bool set)
    {
        collapseParams_.OptimalPlacement = set;
    }  // Default: true
    /**
     * @brief Sets whether or not you want to preserve the topology
     * The default is true.
     * @param set Bool to tell if you want to use this feature
     */
    void setPreserveTopology(bool set)
    {
        collapseParams_.PreserveTopology = set;
    }  // Default: true
    /**
     * @brief Sets whether or not you want to preserve the boundary
     * The default is true. The decimation will preserve the boundary
     * of the mesh.
     * @param set Bool to tell if you want to use this feature.
     */
    void setPreserveBoundary(bool set)
    {
        collapseParams_.PreserveBoundary = set;
    }  // Default: true
    /**
     * @brief Sets the error boundary
     * The default is 1e-15
     * @param epsilon Desired error boundary
     */
    void setQuadricEpsilon(double epsilon)
    {
        collapseParams_.QuadricEpsilon = epsilon;
    }  // Default:1e-15
    /**
     * @brief Sets whether or not you want to check the quailty of the mesh
     * The default is true.
     * @param set Bool to tell if you want to use this feature.
     */
    void setQualityCheck(bool set)
    {
        collapseParams_.QualityCheck = set;
    }  // Default: true
    /**
     * @brief Sets whether or not to use the quadric to determine the quality
     * The default is false.
     * @param set Bool to tell if you want to use this feature
     */
    void setQualityQuadric(bool set)
    {
        collapseParams_.QualityQuadric = set;
    }  // Default: false
    /**
     * @brief Sets the quality threshold
     * The default is .3.
     * @param thr Desired quality threshold
     */
    void setQualityThr(double thr)
    {
        collapseParams_.QualityThr = thr;
    }  // Default: .3
    /**
     * @brief Sets whether or not to give the quality weight
     * The default is false.
     * @param set Bool to tell if you want to use this feature
     */
    void setQualityWeight(bool set)
    {
        collapseParams_.QualityWeight = set;
    }  // Default: false
    /**
     * @brief Sets the quality weight factor
     * This only matters if using QuailtyWeight.
     * The default is 100.
     * @param factor Desired weight factor for quality
     */
    void setQualityWeightFactor(double factor)
    {
        collapseParams_.QualityWeight = factor;
    }  // Default: 100.0
    /**
     * @brief Sets the factor by which to scale the mesh
     * The default is 1.
     * @param scale Desired scale factor of the mesh
     */
    void setScaleFactor(double scale)
    {
        collapseParams_.ScaleFactor = scale;
    }  // Default: 1.0
    /**
     * @brief Sets wheter the scale is independent of other factors
     * The default is true.
     * @param set Bool to tell if you want to use this feature
     */
    void setScaleIndependent(bool set)
    {
        collapseParams_.ScaleIndependent = set;
    }  // Default: true
    /**
     * @brief Sets whether or not to use Area to determine edge collapse
     * The default is true.
     * @param set Bool to tell if you want to use this feature
     */
    void setUseArea(bool set)
    {
        collapseParams_.UseArea = set;
    }  // Default: true;
    /**
     * @brief Sets wheter to use Vertex Weight to determine edge collapse
     * The default is false.
     * @param set Bool to tell if you want to use this feature
     */
    void setUseVertexWeight(bool set)
    {
        collapseParams_.UseVertexWeight = set;
    }  // Default: false
    //@}

    // Processing
    /**
     * @brief Computes the edge collapse
     * This uses the default number of desired faces.
     * If the desired number of faces has not been set,
     * the algorithm runs until the error is minimized.
     */
    void compute();

    /**
     * @brief Computes the edge collapse
     * This computes the edge collapse using the desired
     * number of faces to the number that was passed in.
     * @param desiredFaces Number of faces you want in the new mesh
     */
    void compute(size_t desiredFaces);

    // Output
    /**
     * @brief Returns a pointer to the decimated mesh
     * @return Pointer to an ITK Mesh which has been decimated
     */
    ITKMesh::Pointer getMesh();

private:
    /**
     * @brief Converts the ITK mesh to a vcgMesh
     * This takes in the ITK mesh passed in and
     * converts it to an vcgMesh so that the collapse
     * operations can be performed.
     */
    void convertMeshtoVCG_();
    /** Pointer to an ITK Mesh that needs to be decimated*/
    ITKMesh::Pointer itkInput_;
    /** Converted ITK Mesh that needs to be decimated*/
    VcgMesh vcgInput_;
    /** Pointer to an ITK Mesh which has been decimated */
    ITKMesh::Pointer outputMesh_;
    /** Desired number of faces in the new mesh */
    size_t desiredFaces_;
    /** The parameters for the edge collapse*/
    vcg::tri::TriEdgeCollapseQuadricParameter collapseParams_;

};  // QuadricEdgeCollapse

}  // meshing
}  // volcart
