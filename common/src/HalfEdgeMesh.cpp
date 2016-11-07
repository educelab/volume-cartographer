//
// Created by Seth Parker on 6/17/16.
//

#include "common/types/HalfEdgeMesh.h"

double MINANGLE = M_PI / 180.0;
double MAXANGLE = M_PI - MINANGLE;

using namespace volcart;
///// Constructors & Destructors /////
HalfEdgeMesh::HalfEdgeMesh(){};
HalfEdgeMesh::~HalfEdgeMesh() { clear(); };

void HalfEdgeMesh::clear()
{
    _verts.clear();
    _edges.clear();
    _faces.clear();
    _interior.clear();
    _boundary.clear();
}

///// Mesh Access /////
// Add a vertex at XYZ
HalfEdgeMesh::VertPtr HalfEdgeMesh::addVert(double x, double y, double z)
{
    HalfEdgeMesh::VertPtr v = std::make_shared<HalfEdgeMesh::Vert>();
    v->id = _verts.size();
    if (_verts.size() > 0)
        _verts.back()->nextlink = v;

    v->xyz = cv::Vec3d(x, y, z);
    v->uv = cv::Vec2d(0, 0);
    v->lambdaPlanar = 0.0;
    v->lambdaLength = 1.0;

    _verts.push_back(v);

    return v;
}

// Add a face connecting v0 -> v1 -> v2
HalfEdgeMesh::FacePtr HalfEdgeMesh::addFace(IDType v0, IDType v1, IDType v2)
{
    // Make the faces, edges, and angles
    HalfEdgeMesh::FacePtr f = std::make_shared<Face>();
    HalfEdgeMesh::EdgePtr e0 = std::make_shared<Edge>();
    HalfEdgeMesh::EdgePtr e1 = std::make_shared<Edge>();
    HalfEdgeMesh::EdgePtr e2 = std::make_shared<Edge>();
    HalfEdgeMesh::AnglePtr a0 = std::make_shared<Angle>();
    HalfEdgeMesh::AnglePtr a1 = std::make_shared<Angle>();
    HalfEdgeMesh::AnglePtr a2 = std::make_shared<Angle>();

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
    e0->vert = _verts[v0];
    e1->vert = _verts[v1];
    e2->vert = _verts[v2];

    // Link the vertices to their edges if they dont have one
    if (e0->vert->edge == nullptr)
        e0->vert->edge = e0;
    if (e1->vert->edge == nullptr)
        e1->vert->edge = e1;
    if (e2->vert->edge == nullptr)
        e2->vert->edge = e2;

    // Compute the current angles
    cv::Vec3d a;
    a[0] = _angle(e0->vert->xyz, e1->vert->xyz, e2->vert->xyz);
    a[1] = _angle(e1->vert->xyz, e2->vert->xyz, e0->vert->xyz);
    a[2] = _angle(e2->vert->xyz, e0->vert->xyz, e1->vert->xyz);

    // Clamping
    for (int i = 0; i < 3; ++i) {
        if (a[i] < MINANGLE)
            a[i] = MINANGLE;
        else if (a[i] > MAXANGLE)
            a[i] = MAXANGLE;
    }

    // Assign the most recent values
    a0->alpha = a0->beta = a0->phi = a[0];
    a1->alpha = a1->beta = a1->phi = a[1];
    a2->alpha = a2->beta = a2->phi = a[2];
    a0->weight = 1.0 / (a[0] * a[0]);
    a1->weight = 1.0 / (a[1] * a[1]);
    a2->weight = 1.0 / (a[2] * a[2]);

    // Add everything to their respective arrays
    e0->id = _edges.size();
    if (_edges.size() > 0)
        _edges.back()->nextlink = e0;
    _edges.push_back(e0);

    e1->id = _edges.size();
    _edges.back()->nextlink = e1;
    _edges.push_back(e1);

    e2->id = _edges.size();
    _edges.back()->nextlink = e2;
    _edges.push_back(e2);

    f->id = _faces.size();
    f->lambdaTriangle = 0.0;
    if (_faces.size() > 0)
        _faces.back()->nextlink = f;
    f->connected = false;
    _faces.push_back(f);

    return f;
}

///// Vertex Access /////
// Get a vertex by vertex id
HalfEdgeMesh::VertPtr HalfEdgeMesh::getVert(IDType id) const
{
    return _verts[id];
};
// Get reference to the vertices begin iterator
std::vector<HalfEdgeMesh::VertPtr>::iterator HalfEdgeMesh::getVertsBegin()
{
    return _verts.begin();
};
// Get reference to the vertices end iterator
std::vector<HalfEdgeMesh::VertPtr>::iterator HalfEdgeMesh::getVertsEnd()
{
    return _verts.end();
};

///// Edge Access /////
// Get an edge by edge id
HalfEdgeMesh::EdgePtr HalfEdgeMesh::getEdge(IDType id) const
{
    return _edges[id];
};
// Get reference to the edges begin iterator
std::vector<HalfEdgeMesh::EdgePtr>::iterator HalfEdgeMesh::getEdgesBegin()
{
    return _edges.begin();
};
// Get reference to the edges end iterator
std::vector<HalfEdgeMesh::EdgePtr>::iterator HalfEdgeMesh::getEdgesEnd()
{
    return _edges.end();
};

///// Face Access /////
// Get a face by face id
HalfEdgeMesh::FacePtr HalfEdgeMesh::getFace(IDType id) const
{
    return _faces[id];
};
// Get reference to the vertices begin iterator
std::vector<HalfEdgeMesh::FacePtr>::iterator HalfEdgeMesh::getFacesBegin()
{
    return _faces.begin();
};
// Get reference to the vertices end iterator
std::vector<HalfEdgeMesh::FacePtr>::iterator HalfEdgeMesh::getFacesEnd()
{
    return _faces.end();
};

///// Interior/Boundary Access /////
// Get reference to the interior vertices begin iterator
std::vector<HalfEdgeMesh::VertPtr>::iterator HalfEdgeMesh::getInteriorBegin()
{
    return _interior.begin();
};
// Get reference to the interior vertices end iterator
std::vector<HalfEdgeMesh::VertPtr>::iterator HalfEdgeMesh::getInteriorEnd()
{
    return _interior.end();
};
// Get reference to the boundary vertices begin iterator
std::vector<HalfEdgeMesh::VertPtr>::iterator HalfEdgeMesh::getBoundaryBegin()
{
    return _boundary.begin();
};
// Get reference to the boundary vertices end iterator
std::vector<HalfEdgeMesh::VertPtr>::iterator HalfEdgeMesh::getBoundaryEnd()
{
    return _boundary.end();
};

///// Sizes //////
// Get size of the vertices vector
size_t HalfEdgeMesh::getNumberOfVerts() const { return _verts.size(); };
// Get size of the edges vector
size_t HalfEdgeMesh::getNumberOfEdges() const { return _edges.size(); };
// Get size of the faces vector
size_t HalfEdgeMesh::getNumberOfFaces() const { return _faces.size(); };
// Get size of the interior vector
size_t HalfEdgeMesh::getNumberOfInteriorPoints() const
{
    return _interior.size();
};
// Get size of the boundary vector
size_t HalfEdgeMesh::getNumberOfBoundaryPoints() const
{
    return _boundary.size();
};

///// Special Construction Tasks /////
void HalfEdgeMesh::constructConnectedness()
{
    // Connect pairs
    _connectAllPairs();

    // Connect boundaries
    _computeBoundary();
}

// Connect the edges that have the same vertices as end points
void HalfEdgeMesh::_connectAllPairs()
{
    HalfEdgeMesh::EdgePtr e0, e1, e2;

    for (auto f = _faces[0]; f; f = f->nextlink) {
        if (f->connected)
            continue;

        e0 = f->edge;
        e1 = e0->next;
        e2 = e1->next;

        // If we don't find a pair, set the vert's only edge to this one
        e0->pair = _findEdgePair(e0->vert->id, e1->vert->id);
        if (e0->pair != nullptr)
            e0->pair->pair = e0;
        else
            e0->vert->edge = e0;
        e1->pair = _findEdgePair(e1->vert->id, e2->vert->id);
        if (e1->pair != nullptr)
            e1->pair->pair = e1;
        else
            e1->vert->edge = e1;
        e2->pair = _findEdgePair(e2->vert->id, e0->vert->id);
        if (e2->pair != nullptr)
            e2->pair->pair = e2;
        else
            e2->vert->edge = e2;

        f->connected = true;
    }
}

// Find the other edge that shares the same two vertices
HalfEdgeMesh::EdgePtr HalfEdgeMesh::_findEdgePair(
    HalfEdgeMesh::IDType A, HalfEdgeMesh::IDType B)
{
    HalfEdgeMesh::EdgePtr pair = nullptr;
    // Check these id's against each edge
    for (auto e = _edges[0]; e; e = e->nextlink) {
        auto f_id = e->face->id;
        // If the current edge's first if matches this one's B, and vice versa,
        // it's the pair we want
        if ((e->vert->id == B) && (e->next->vert->id == A)) {
            pair = e;
            break;
        }
    }

    return pair;
}

// Compute which edges are boundaries and which are interior
void HalfEdgeMesh::_computeBoundary()
{
    for (auto v = _verts[0]; v; v = v->nextlink) {
        if (v->interior())
            _interior.push_back(v);
        else
            _boundary.push_back(v);
    }
}

///// Topology Traversal /////
HalfEdgeMesh::EdgePtr HalfEdgeMesh::nextWheelEdge(EdgePtr e)
{
    return e->next->next->pair;
};

HalfEdgeMesh::EdgePtr HalfEdgeMesh::prevWheelEdge(EdgePtr e)
{
    return (e->pair) ? e->pair->next : nullptr;
};

HalfEdgeMesh::EdgePtr HalfEdgeMesh::nextBoundaryEdge(EdgePtr e)
{
    return e->next->vert->edge;
};

HalfEdgeMesh::EdgePtr HalfEdgeMesh::prevBoundaryEdge(EdgePtr e)
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
double HalfEdgeMesh::_angle(cv::Vec3d A, cv::Vec3d B, cv::Vec3d C)
{

    cv::Vec3d vec_1 = B - A;
    cv::Vec3d vec_2 = C - A;

    cv::normalize(vec_1, vec_1);
    cv::normalize(vec_2, vec_2);

    double dot = vec_1.dot(vec_2);

    if (dot <= -1.0f)
        return M_PI;
    else if (dot >= 1.0f)
        return 0.0f;
    else
        return acos(dot);
}
