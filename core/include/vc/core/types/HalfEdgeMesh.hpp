// Created by Seth Parker on 6/17/16.
// Mostly copied from the HE structure used by Blender's parameterization
// A half-edge mesh stores a unique edge for each edge of each face.
// e.g. If two triangles share vertices A and B, this will result in two edges:
// edge AB and edge BA
#pragma once

#include <memory>

#include <opencv2/core.hpp>

#include "vc/core/vc_defines.hpp"

namespace volcart
{
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

    struct Vert {
        Vert() : id{}, pid{}, lambdaPlanar{}, lambdaLength{} {}

        VertPtr nextLink;
        IDType id;
        ITKMesh::PointIdentifier pid;  // Original point ID in ITK mesh

        EdgePtr edge;
        cv::Vec3d xyz;
        cv::Vec2d uv;

        double lambdaPlanar;
        double lambdaLength;

        bool interior() const { return edge->pair != nullptr; }
    };

    struct Edge {
        Edge() : id{} {}

        EdgePtr nextLink;
        IDType id;

        VertPtr vert;  // Starting index
        AnglePtr angle;
        EdgePtr pair;  // Parallel edge. If this is edge AB, pair is edge BA
        EdgePtr next;  // Next edge in the face
        FacePtr face;
    };

    // Topology traversal
    // The wheel is the set of edges that surround a single vertex
    EdgePtr nextWheelEdge(const EdgePtr& e) { return e->next->next->pair; }
    EdgePtr prevWheelEdge(const EdgePtr& e)
    {
        return (e->pair) ? e->pair->next : nullptr;
    }
    EdgePtr nextBoundaryEdge(const EdgePtr& e) { return e->next->vert->edge; }
    EdgePtr prevBoundaryEdge(const EdgePtr& e);

    struct Angle {
        Angle() : alpha{}, beta{}, phi{}, weight{}, bAlpha{}, sine{}, cosine{}
        {
        }

        EdgePtr edge;  // The edge that owns this angle

        double alpha;   // Current angle
        double beta;    // Original angle
        double phi;     // Ideal angle
        double weight;  // Typically 1/b^2

        double bAlpha;  // current gradient of the ABF constraints

        double sine;
        double cosine;
    };

    struct Face {
        Face()
            : id{}
            , cid{}
            , lambdaTriangle{}
            , bstar{}
            , dstar{}
            , bTriangle{}
            , connected{}
        {
        }

        FacePtr nextLink;
        IDType id;
        ITKMesh::CellIdentifier cid;  // Original cell ID in ITK mesh
        EdgePtr edge;                 // First edge of the face
        double lambdaTriangle;
        double bstar;
        double dstar;
        double bTriangle;
        bool connected;
    };

public:
    HalfEdgeMesh() {}
    ~HalfEdgeMesh() { clear(); }

    void clear();

    ///// Mesh Access /////
    VertPtr addVert(double x, double y, double z);
    FacePtr addFace(IDType v0, IDType v1, IDType v2);

    VertPtr getVert(IDType id) const { return verts_[id]; }
    EdgePtr getEdge(IDType id) const { return edges_[id]; }
    FacePtr getFace(IDType id) const { return faces_[id]; }

    std::vector<VertPtr>::iterator getVertsBegin()
    {
        return std::begin(verts_);
    }
    std::vector<VertPtr>::iterator getVertsEnd() { return std::end(verts_); }
    std::vector<EdgePtr>::iterator getEdgesBegin()
    {
        return std::begin(edges_);
    }
    std::vector<EdgePtr>::iterator getEdgesEnd() { return std::end(edges_); }
    std::vector<FacePtr>::iterator getFacesBegin()
    {
        return std::begin(faces_);
    }
    std::vector<FacePtr>::iterator getFacesEnd() { return std::end(faces_); }

    std::vector<VertPtr>::iterator getInteriorBegin()
    {
        return std::begin(interior_);
    }
    std::vector<VertPtr>::iterator getInteriorEnd()
    {
        return std::end(interior_);
    }
    std::vector<VertPtr>::iterator getBoundaryBegin()
    {
        return std::begin(boundary_);
    }
    std::vector<VertPtr>::iterator getBoundaryEnd()
    {
        return std::end(boundary_);
    }

    size_t getNumberOfVerts() const { return verts_.size(); }
    size_t getNumberOfEdges() const { return edges_.size(); }
    size_t getNumberOfFaces() const { return faces_.size(); }
    size_t getNumberOfInteriorPoints() const { return interior_.size(); }
    size_t getNumberOfBoundaryPoints() const { return boundary_.size(); }

    ///// Special Construction Tasks /////
    void constructConnectedness();

private:
    std::vector<VertPtr> verts_;
    std::vector<EdgePtr> edges_;
    std::vector<FacePtr> faces_;

    std::vector<VertPtr> interior_;
    std::vector<VertPtr> boundary_;

    ///// Special Construction Tasks /////
    void connect_all_pairs_();
    HalfEdgeMesh::EdgePtr find_edge_pair_(
        HalfEdgeMesh::IDType a, HalfEdgeMesh::IDType b);
    void compute_boundary_();

    ///// Math functions /////
    double angle_(const cv::Vec3d& a, const cv::Vec3d& b, const cv::Vec3d& c);
};
}
