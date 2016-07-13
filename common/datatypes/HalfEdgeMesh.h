//
// Created by Seth Parker on 6/17/16.
// Mostly copied from the HE structure used by Blender's parameterization

#ifndef VC_HALFEDGEMESH_H
#define VC_HALFEDGEMESH_H

#include <iostream>
#include <exception>
#include <memory>

#include <opencv2/opencv.hpp>

#include "../vc_defines.h"
#include "../vc_datatypes.h"

namespace volcart {
    class HalfEdgeMesh {
    public:
        class Vert;
        class Edge;
        class Angle;
        class Face;

        typedef std::shared_ptr <Vert>  VertPtr;
        typedef std::shared_ptr <Edge>  EdgePtr;
        typedef std::shared_ptr <Angle> AnglePtr;
        typedef std::shared_ptr <Face>  FacePtr;

        typedef unsigned long IDType;

        class Vert {
        public:
            Vert(){};

            VertPtr nextlink;
            IDType id;
            VC_MeshType::PointIdentifier pid; // Original point ID in ITK mesh

            EdgePtr edge;
            cv::Vec3d xyz;
            cv::Vec2d uv;

            double lambdaPlanar;
            double lambdaLength;

            bool interior() { return edge->pair != nullptr; };
        };

        class Edge {
        public:
            Edge(){};

            EdgePtr nextlink;
            IDType id;

            VertPtr  vert;
            AnglePtr angle;
            EdgePtr  pair;
            EdgePtr  next;
            FacePtr  face;
        };

        // Topology traversal
        EdgePtr nextWheelEdge(EdgePtr e);
        EdgePtr prevWheelEdge(EdgePtr e);
        EdgePtr nextBoundaryEdge(EdgePtr e);
        EdgePtr prevBoundaryEdge(EdgePtr e);

        class Angle {
        public:
            Angle(){};

            EdgePtr edge;

            double alpha;  // Current angle
            double beta;   // Ideal/Original angle
            double weight; // Typically 1/b^2

            double bAlpha; // current gradient of the ABF constraints

            double sine;
            double cosine;
        };

        class Face {
        public:
            Face(){};
            FacePtr nextlink;
            IDType id;
            VC_MeshType::CellIdentifier cid; // Original cell ID in ITK mesh

            EdgePtr edge;

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
        VertPtr addVert( double x, double y, double z );
        FacePtr addFace( IDType v0, IDType v1, IDType v2 );

        VertPtr getVert( IDType id );
        EdgePtr getEdge( IDType id );
        FacePtr getFace( IDType id );

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

        size_t getNumberOfVerts();
        size_t getNumberOfEdges();
        size_t getNumberOfFaces();
        size_t getNumberOfInteriorPoints();
        size_t getNumberOfBoundaryPoints();

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
        HalfEdgeMesh::EdgePtr _findEdgePair( HalfEdgeMesh::IDType A, HalfEdgeMesh::IDType B );
        void _computeBoundary();

        ///// Math functions /////
        double _angle(cv::Vec3d A, cv::Vec3d B, cv::Vec3d C);

    };
}


#endif //VC_HALFEDGEMESH_H
