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

#include <memory>

#include <opencv2/core.hpp>

#include "vc/core/types/ITKMesh.hpp"

namespace volcart
{
/**
 * @class HalfEdgeMesh
 * @author Seth Parker
 * @date 6/17/16
 *
 * @copyright This implementation is ported from the implementation in Blender,
 * and as such is subject to the terms of the GPL v2.
 *
 * @brief A triangular mesh structure with bidirectional connections along
 * edges
 *
 * A half-edge mesh stores a unique edge for each triangle that uses a pair of
 * connected vertices. e.g. If two triangles share an edge defined by A and B,
 * there will be two stored Edge objects: \f$\overline{AB}\f$ and
 * \f$\overline{BA}\f$. This property is useful for mesh traversal.
 *
 * Used primarily by AngleBasedFlattening.
 *
 * @ingroup Types
 */
class HalfEdgeMesh
{
public:
    struct Vert;
    struct Edge;
    struct Angle;
    struct Face;

    using VertPtr = std::shared_ptr<Vert>;
    using EdgePtr = std::shared_ptr<Edge>;
    using AnglePtr = std::shared_ptr<Angle>;
    using FacePtr = std::shared_ptr<Face>;

    using IDType = uint64_t;

    /** @brief %Vertex */
    struct Vert {
        /** Constructor */
        Vert() = default;

        /** Next connected vertex */
        VertPtr nextLink;

        /** Vertex ID number */
        IDType id;

        /** Original vertex ID in the ITK mesh */
        ITKMesh::PointIdentifier pid;

        /** First edge that begins with this vertex */
        EdgePtr edge;

        /** 3D vertex position */
        cv::Vec3d xyz;

        /** UV vertex position */
        cv::Vec2d uv;

        /** ABF planarity constraint */
        double lambdaPlanar;

        /** ABF edge length constraint */
        double lambdaLength;

        /** @brief Returns true if the vertex is interior to the graph */
        bool interior() const { return edge->pair != nullptr; }
    };

    /** @brief %Edge between two vertices */
    struct Edge {
        /** Constructor */
        Edge() = default;

        /** Next connected Edge. Not necessarily in the same Face. */
        EdgePtr nextLink;

        /** %Edge ID number */
        IDType id;

        /** Starting Vert in the edge */
        VertPtr vert;

        /** Measure of the interior angle adjacent to this edge (at the starting
         * vertex) */
        AnglePtr angle;

        /** The mirror Edge to this one, from the adjacent Face */
        EdgePtr pair;

        /** The next Edge in the triangle */
        EdgePtr next;

        /** The Face that owns this Edge */
        FacePtr face;
    };

    /**@{*/
    /** @brief Return the next Edge in the mesh that shares the same starting
     * vertex as the provided Edge
     */
    EdgePtr nextWheelEdge(const EdgePtr& e) { return e->next->next->pair; }

    /** @brief Return the previous Edge in the mesh that shares the same
     * starting vertex as the provided Edge
     */
    EdgePtr prevWheelEdge(const EdgePtr& e)
    {
        return (e->pair) ? e->pair->next : nullptr;
    }

    /** @brief Return the next Edge along the boundary of the mesh */
    EdgePtr nextBoundaryEdge(const EdgePtr& e) { return e->next->vert->edge; }
    /** @brief Return the previous Edge along the boundary of the mesh */
    EdgePtr prevBoundaryEdge(const EdgePtr& e);
    /**@}*/

    /** @brief Interior angle of a triangle */
    struct Angle {
        /** Constructor */
        Angle() = default;

        /** Edge that owns this angle */
        EdgePtr edge;

        /** ABF "current" angle measure */
        double alpha;

        /** ABF starting angle measure */
        double beta;

        /** ABF ideal angle measure */
        double phi;

        /** ABF angle weight factor */
        double weight;

        /** ABF "current" gradient */
        double bAlpha;

        /** Sine of the alpha angle */
        double sine;

        /** Cosine of the alpha angle */
        double cosine;
    };

    /** @brief Triangular face */
    struct Face {
        /** Constructor */
        Face() = default;

        /** Next Face in the mesh */
        FacePtr nextLink;

        /** %Face ID number */
        IDType id;

        /** Original cell ID in the ITK mesh */
        ITKMesh::CellIdentifier cid;

        /** First Edge of the Face */
        EdgePtr edge;

        /** ABF triangle weight penalty */
        double lambdaTriangle;

        /** ABF Parameter */
        double bstar;

        /** ABF Parameter */
        double dstar;

        /** ABF Parameter */
        double bTriangle;

        /** Indicates whether Face and its Edges have been linked */
        bool connected;
    };

public:
    /**@{*/
    HalfEdgeMesh() = default;
    ~HalfEdgeMesh() { clear(); }

    /** @brief Empty the mesh of all vertices, faces, and edges */
    void clear();
    /**@}*/

    /**@{*/
    /**
     * @brief Add a Vert to the mesh
     * @return Pointer to the newly created Vert
     */
    VertPtr addVert(double x, double y, double z);

    /**
     * @brief Add a Face to the mesh using Vert ID's
     *
     * Vertices first need to be added using addVert().
     *
     * @return Pointer to the newly created Face
     */
    FacePtr addFace(IDType v0, IDType v1, IDType v2);

    /** @brief Get Vert by ID */
    VertPtr getVert(IDType id) const { return verts_[id]; }

    /** @brief Get Edge by ID */
    EdgePtr getEdge(IDType id) const { return edges_[id]; }

    /** @brief Get Face by ID */
    FacePtr getFace(IDType id) const { return faces_[id]; }

    /** @brief Get iterator to the beginning of the internal Vert list */
    std::vector<VertPtr>::iterator getVertsBegin()
    {
        return std::begin(verts_);
    }
    /** @brief Get iterator to the end of the internal Vert list */
    std::vector<VertPtr>::iterator getVertsEnd() { return std::end(verts_); }

    /** @brief Get iterator to the beginning of the internal Edge list */
    std::vector<EdgePtr>::iterator getEdgesBegin()
    {
        return std::begin(edges_);
    }

    /** @brief Get iterator to the end of the internal Edge list */
    std::vector<EdgePtr>::iterator getEdgesEnd() { return std::end(edges_); }

    /** @brief Get iterator to the beginning of the internal Face list */
    std::vector<FacePtr>::iterator getFacesBegin()
    {
        return std::begin(faces_);
    }

    /** @brief Get iterator to the end of the internal Face list */
    std::vector<FacePtr>::iterator getFacesEnd() { return std::end(faces_); }

    /** @brief Get iterator to the beginning of the interior Vert list */
    std::vector<VertPtr>::iterator getInteriorBegin()
    {
        return std::begin(interior_);
    }

    /** @brief Get iterator to the end of the interior Vert list */
    std::vector<VertPtr>::iterator getInteriorEnd()
    {
        return std::end(interior_);
    }

    /** @brief Get iterator to the beginning of the boundary Vert list */
    std::vector<VertPtr>::iterator getBoundaryBegin()
    {
        return std::begin(boundary_);
    }

    /** @brief Get iterator to the end of the boundary Vert list */
    std::vector<VertPtr>::iterator getBoundaryEnd()
    {
        return std::end(boundary_);
    }

    /** @brief Get the number of vertices in the mesh */
    size_t getNumberOfVerts() const { return verts_.size(); }

    /** @brief Get the number of edges in the mesh */
    size_t getNumberOfEdges() const { return edges_.size(); }

    /** @brief Get the number of faces in the mesh */
    size_t getNumberOfFaces() const { return faces_.size(); }

    /** @brief Get the number of interior vertices */
    size_t getNumberOfInteriorPoints() const { return interior_.size(); }

    /** @brief Get the number of exterior vertices */
    size_t getNumberOfBoundaryPoints() const { return boundary_.size(); }
    /**@}*/

    /**@{*/
    /**
     * @brief Connect Face, Vert, and Edge links
     *
     * This function makes sure that all Vert, Face, and Edge pointers in the
     * mesh are properly set. It also computes which edges are boundary edges
     * and which are interior edges.
     *
     * @warning This function should be called after all vertices and faces have
     * been added to the mesh. Failure to call this function will likely result
     * in problems.
     */
    void constructConnectedness();
    /**@}*/

private:
    /** Vert list */
    std::vector<VertPtr> verts_;
    /** Edge list */
    std::vector<EdgePtr> edges_;
    /** Face list */
    std::vector<FacePtr> faces_;

    /** Interior Vert list */
    std::vector<VertPtr> interior_;
    /** Boundary Vert List */
    std::vector<VertPtr> boundary_;

    /**@{*/
    /** Connect all edges that have the same vertices as end points */
    void connect_all_pairs_();

    /** Find the edge that has vertex end points with the same IDs, but in the
     * opposite order.
     */
    HalfEdgeMesh::EdgePtr find_edge_pair_(
        HalfEdgeMesh::IDType a, HalfEdgeMesh::IDType b);

    /** Compute boundary edges */
    void compute_boundary_();
    /**@}*/

    /** @brief Return the angle between vectors \f$\vec{ab}\f$ and
     * \f$\vec{ac}\f$
     */
    double angle_(const cv::Vec3d& a, const cv::Vec3d& b, const cv::Vec3d& c);
};
}
