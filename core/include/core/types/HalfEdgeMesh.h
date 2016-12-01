// Created by Seth Parker on 6/17/16.
// Mostly copied from the HE structure used by Blender's parameterization
// A half-edge mesh stores a unique edge for each edge of each face.
// e.g. If two triangles share vertices A and B, this will result in two edges:
// edge AB and edge BA
#pragma once

#include <exception>
#include <iostream>
#include <memory>

#include <opencv2/opencv.hpp>

#include "core/vc_defines.h"

namespace volcart
{
class HalfEdgeMesh
{
public:
    class Vert;
    class Edge;
    class Angle;
    class Face;

    using VertPtr = std::shared_ptr<Vert>;
    using EdgePtr = std::shared_ptr<Edge>;
    using AnglePtr = std::shared_ptr<Angle>;
    using FacePtr = std::shared_ptr<Face>;

    using IDType = unsigned long;

    class Vert
    {
    public:
        Vert() : id{}, pid{}, lambdaPlanar{}, lambdaLength{} {}

        VertPtr nextlink;
        IDType id;
        ITKMesh::PointIdentifier pid;  // Original point ID in ITK mesh

        EdgePtr edge;
        cv::Vec3d xyz;
        cv::Vec2d uv;

        double lambdaPlanar;
        double lambdaLength;

        bool interior() const { return edge->pair != nullptr; };
    };

    class Edge
    {
    public:
        Edge() : id{} {}

        EdgePtr nextlink;
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

    class Angle
    {
    public:
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

    class Face
    {
    public:
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
        FacePtr nextlink;
        IDType id;
        ITKMesh::CellIdentifier cid;  // Original cell ID in ITK mesh

        EdgePtr edge;  // First edge of the face

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

    VertPtr getVert(IDType id) const { return _verts[id]; }
    EdgePtr getEdge(IDType id) const { return _edges[id]; }
    FacePtr getFace(IDType id) const { return _faces[id]; }

    std::vector<VertPtr>::iterator getVertsBegin()
    {
        return std::begin(_verts);
    }
    std::vector<VertPtr>::iterator getVertsEnd() { return std::end(_verts); }
    std::vector<EdgePtr>::iterator getEdgesBegin()
    {
        return std::begin(_edges);
    }
    std::vector<EdgePtr>::iterator getEdgesEnd() { return std::end(_edges); }
    std::vector<FacePtr>::iterator getFacesBegin()
    {
        return std::begin(_faces);
    }
    std::vector<FacePtr>::iterator getFacesEnd() { return std::end(_faces); }

    std::vector<VertPtr>::iterator getInteriorBegin()
    {
        return std::begin(_interior);
    }
    std::vector<VertPtr>::iterator getInteriorEnd()
    {
        return std::end(_interior);
    }
    std::vector<VertPtr>::iterator getBoundaryBegin()
    {
        return std::begin(_boundary);
    }
    std::vector<VertPtr>::iterator getBoundaryEnd()
    {
        return std::end(_boundary);
    }

    size_t getNumberOfVerts() const { return _verts.size(); }
    size_t getNumberOfEdges() const { return _edges.size(); }
    size_t getNumberOfFaces() const { return _faces.size(); }
    size_t getNumberOfInteriorPoints() const { return _interior.size(); }
    size_t getNumberOfBoundaryPoints() const { return _boundary.size(); }

    ///// Special Construction Tasks /////
    void constructConnectedness();

private:
    std::vector<VertPtr> _verts;
    std::vector<EdgePtr> _edges;
    std::vector<FacePtr> _faces;

    std::vector<VertPtr> _interior;
    std::vector<VertPtr> _boundary;

    ///// Special Construction Tasks /////
    void _connectAllPairs();
    HalfEdgeMesh::EdgePtr _findEdgePair(
        HalfEdgeMesh::IDType A, HalfEdgeMesh::IDType B);
    void _computeBoundary();

    ///// Math functions /////
    double _angle(const cv::Vec3d& A, const cv::Vec3d& B, const cv::Vec3d& C);
};
}  // namespace volcart
