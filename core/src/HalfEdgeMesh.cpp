//
// Created by Seth Parker on 6/17/16.
//
#include <cmath>

#include "core/types/HalfEdgeMesh.h"

static constexpr double MINANGLE = M_PI / 180.0;
static constexpr double MAXANGLE = M_PI - MINANGLE;

using namespace volcart;

void HalfEdgeMesh::clear()
{
    verts_.clear();
    edges_.clear();
    faces_.clear();
    interior_.clear();
    boundary_.clear();
}

///// Mesh Access /////
// Add a vertex at XYZ
HalfEdgeMesh::VertPtr HalfEdgeMesh::addVert(double x, double y, double z)
{
    auto v = std::make_shared<HalfEdgeMesh::Vert>();
    v->id = verts_.size();
    if (!verts_.empty()) {
        verts_.back()->nextLink = v;
    }

    v->xyz = cv::Vec3d{x, y, z};
    v->uv = cv::Vec2d{0, 0};
    v->lambdaPlanar = 0.0;
    v->lambdaLength = 1.0;

    verts_.push_back(v);

    return v;
}

// Add a face connecting v0 -> v1 -> v2
HalfEdgeMesh::FacePtr HalfEdgeMesh::addFace(IDType v0, IDType v1, IDType v2)
{
    // Make the faces, edges, and angles
    auto f = std::make_shared<Face>();
    auto e0 = std::make_shared<Edge>();
    auto e1 = std::make_shared<Edge>();
    auto e2 = std::make_shared<Edge>();
    auto a0 = std::make_shared<Angle>();
    auto a1 = std::make_shared<Angle>();
    auto a2 = std::make_shared<Angle>();

    // Link the edges to the face
    f->edge = e0;
    e0->face = e1->face = e2->face = f;

    // Link the edges to each other
    e0->next = e1;
    e1->next = e2;
    e2->next = e0;

    // Link the edges and angles
    e0->angle = a0;
    a0->edge = e0;
    e1->angle = a1;
    a1->edge = e1;
    e2->angle = a2;
    a2->edge = e2;

    // Link the edges to their vertices
    e0->vert = verts_[v0];
    e1->vert = verts_[v1];
    e2->vert = verts_[v2];

    // Link the vertices to their edges if they dont have one
    if (e0->vert->edge == nullptr) {
        e0->vert->edge = e0;
    }
    if (e1->vert->edge == nullptr) {
        e1->vert->edge = e1;
    }
    if (e2->vert->edge == nullptr) {
        e2->vert->edge = e2;
    }

    // Compute the current angles
    cv::Vec3d a;
    a[0] = angle_(e0->vert->xyz, e1->vert->xyz, e2->vert->xyz);
    a[1] = angle_(e1->vert->xyz, e2->vert->xyz, e0->vert->xyz);
    a[2] = angle_(e2->vert->xyz, e0->vert->xyz, e1->vert->xyz);

    // Clamping
    for (int i = 0; i < 3; ++i) {
        if (a[i] < MINANGLE) {
            a[i] = MINANGLE;
        } else if (a[i] > MAXANGLE) {
            a[i] = MAXANGLE;
        }
    }

    // Assign the most recent values
    a0->alpha = a0->beta = a0->phi = a[0];
    a1->alpha = a1->beta = a1->phi = a[1];
    a2->alpha = a2->beta = a2->phi = a[2];
    a0->weight = 1.0 / (a[0] * a[0]);
    a1->weight = 1.0 / (a[1] * a[1]);
    a2->weight = 1.0 / (a[2] * a[2]);

    // Add everything to their respective arrays
    e0->id = edges_.size();
    if (!edges_.empty()) {
        edges_.back()->nextLink = e0;
    }
    edges_.push_back(e0);

    e1->id = edges_.size();
    edges_.back()->nextLink = e1;
    edges_.push_back(e1);

    e2->id = edges_.size();
    edges_.back()->nextLink = e2;
    edges_.push_back(e2);

    f->id = faces_.size();
    f->lambdaTriangle = 0.0;
    if (!faces_.empty()) {
        faces_.back()->nextLink = f;
    }
    f->connected = false;
    faces_.push_back(f);

    return f;
}

///// Special Construction Tasks /////
void HalfEdgeMesh::constructConnectedness()
{
    // Connect pairs
    connectAllPairs_();

    // Connect boundaries
    computeBoundary_();
}

// Connect the edges that have the same vertices as end points
void HalfEdgeMesh::connectAllPairs_()
{
    HalfEdgeMesh::EdgePtr e0, e1, e2;

    for (auto f = faces_[0]; f; f = f->nextLink) {
        if (f->connected) {
            continue;
        }

        e0 = f->edge;
        e1 = e0->next;
        e2 = e1->next;

        // If we don't find a pair, set the vert's only edge to this one
        e0->pair = findEdgePair_(e0->vert->id, e1->vert->id);
        if (e0->pair != nullptr) {
            e0->pair->pair = e0;
        } else {
            e0->vert->edge = e0;
        }
        e1->pair = findEdgePair_(e1->vert->id, e2->vert->id);
        if (e1->pair != nullptr) {
            e1->pair->pair = e1;
        } else {
            e1->vert->edge = e1;
        }
        e2->pair = findEdgePair_(e2->vert->id, e0->vert->id);
        if (e2->pair != nullptr) {
            e2->pair->pair = e2;
        } else {
            e2->vert->edge = e2;
        }

        f->connected = true;
    }
}

// Find the other edge that shares the same two vertices
HalfEdgeMesh::EdgePtr HalfEdgeMesh::findEdgePair_(
    HalfEdgeMesh::IDType a, HalfEdgeMesh::IDType b)
{
    HalfEdgeMesh::EdgePtr pair = nullptr;
    // Check these id's against each edge
    for (auto e = edges_[0]; e; e = e->nextLink) {
        // If the current edge's first if matches this one's B, and vice versa,
        // it's the pair we want
        if ((e->vert->id == b) && (e->next->vert->id == a)) {
            pair = e;
            break;
        }
    }

    return pair;
}

// Compute which edges are boundaries and which are interior
void HalfEdgeMesh::computeBoundary_()
{
    for (auto v = verts_[0]; v; v = v->nextLink) {
        if (v->interior()) {
            interior_.push_back(v);
        } else {
            boundary_.push_back(v);
        }
    }
}

HalfEdgeMesh::EdgePtr HalfEdgeMesh::prevBoundaryEdge(const EdgePtr& e)
{
    HalfEdgeMesh::EdgePtr we = e, last;

    do {
        last = we;
        we = nextWheelEdge(we);
    } while (we && (we != e));

    return last->next->next;
}

///// Math Functions /////
// Returns the angle between ab and ac
double HalfEdgeMesh::angle_(
    const cv::Vec3d& a, const cv::Vec3d& b, const cv::Vec3d& c)
{
    cv::Vec3d vec1 = b - a;
    cv::Vec3d vec2 = c - a;

    cv::normalize(vec1, vec1);
    cv::normalize(vec2, vec2);

    double dot = vec1.dot(vec2);

    if (dot <= -1.0f) {
        return M_PI;
    } else if (dot >= 1.0f) {
        return 0.0f;
    } else {
        return acos(dot);
    }
}
