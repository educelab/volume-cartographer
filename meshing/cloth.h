//
// Created by Abigail Coleman 2/3/16
//

#ifndef VC_CLOTH_H
#define VC_CLOTH_H

#include <iostream>
#include <math.h>

#include <opencv2/opencv.hpp>

#include <vtkSmoothPolyDataFilter.h>
#include <vtkPLYReader.h>

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "io/ply2itk.h"
#include "itk2vtk.h"
#include "io/objWriter.h"
#include "deepCopy.h"

// bullet converter
#include "itk2bullet.h"
#include <LinearMath/btVector3.h>
#include <vtkQuadricDecimation.h>

namespace volcart {
    namespace meshing {
        class cloth {
        public:
            struct NodeTarget {
                btVector3 t_pos;
                btScalar  t_stepsize;
            };

            cloth ( VC_MeshType::Pointer inputMesh,
                    VC_MeshType::Pointer decimated,
                    int width,
                    int height,
                    int required_iterations);

            volcart::UVMap _returnUVMap();
            VC_MeshType::Pointer _returnMesh() { return _decimated; };

        private:
            int _process();
            btVector3 _btAverageNormal( btSoftBody* body );
            btScalar _btAverageVelocity( btSoftBody* body );
            double _btSurfaceArea( btSoftBody* body );
            static void _planarizeCornersPreTickCallback(btDynamicsWorld *world, btScalar timeStep);
            static void _emptyPreTickCallback(btDynamicsWorld *world, btScalar timeStep);
            void _expandCorners(float magnitude);
            void _dumpState( VC_MeshType::Pointer toUpdate, btSoftBody* body, std::string suffix = "" );
            void _setIterations(int iterations) { _iterations = iterations; };

            void _setSoftBodyFriction( btSoftBody* body ) { body->m_cfg.kDF = 0.01; }; // Dynamic friction coefficient (0-1] Default: 0.2}

            // variables
            static std::vector< btSoftBody::Node* > _pinnedPoints;
            static std::vector< NodeTarget > _targetPoints;

            VC_MeshType::Pointer _input;
            VC_MeshType::Pointer _decimated;
            btSoftBody* _psb;
            int _iterations;
            int _width;
            int _height;

            volcart::UVMap _uvMap;

            btVector3 _middle;
        };
    }// meshing

} // volcart

#endif // VC_CLOTH_H