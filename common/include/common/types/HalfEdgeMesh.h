//
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

#include "common/vc_defines.h"

namespace volcart
{
class HalfEdgeMesh
{
public:
    class Vert;
    class Edge;
    class Angle;
    class Face;

    typedef std::shared_ptr<Vert> VertPtr;
    typedef std::shared_ptr<Edge> EdgePtr;
    typedef std::shared_ptr<Angle> AnglePtr;
    typedef std::shared_ptr<Face> FacePtr;

    typedef unsigned long IDType;

    class Vert
    {
    public:
        Vert(){};

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
        Edge(){};

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
    EdgePtr nextWheelEdge(EdgePtr e);
    EdgePtr prevWheelEdge(EdgePtr e);
    EdgePtr nextBoundaryEdge(EdgePtr e);
    EdgePtr prevBoundaryEdge(EdgePtr e);

    class Angle
    {
    public:
        Angle(){};

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
        Face(){};
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
    HalfEdgeMesh();
    ~HalfEdgeMesh();

    void clear();

    ///// Mesh Access /////
    VertPtr addVert(double x, double y, double z);
    FacePtr addFace(IDType v0, IDType v1, IDType v2);

    VertPtr getVert(IDType id) const;
    EdgePtr getEdge(IDType id) const;
    FacePtr getFace(IDType id) const;

    std::vector<VertPtr>::iterator getVertsBegin();
    std::vector<VertPtr>::iterator getVertsEnd();
    std::vector<EdgePtr>::iterator getEdgesBegin();
    std::vector<EdgePtr>::iterator getEdgesEnd();
    std::vector<FacePtr>::iterator getFacesBegin();
    std::vector<FacePtr>::iterator getFacesEnd();

    std::vector<VertPtr>::iterator getInteriorBegin();
    std::vector<VertPtr>::iterator getInteriorEnd();
    std::vector<VertPtr>::iterator getBoundaryBegin();
    std::vector<VertPtr>::iterator getBoundaryEnd();

    size_t getNumberOfVerts() const;
    size_t getNumberOfEdges() const;
    size_t getNumberOfFaces() const;
    size_t getNumberOfInteriorPoints() const;
    size_t getNumberOfBoundaryPoints() const;

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
    double _angle(cv::Vec3d A, cv::Vec3d B, cv::Vec3d C);
};
}
