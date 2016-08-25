//
// Created by Hannah Hatch on 8/23/16.
//

#pragma once

#include "common/vc_defines.h"
#include "common/types/PointSet.h"
#include "common/types/Point.h"

namespace volcart{
    namespace meshing{

        class OrderedPointSetMesher {
        public:
            //Initalizers
            OrderedPointSetMesher();
            OrderedPointSetMesher(PointSet<Point3d> points);

            // I/O
            void setPointSet(PointSet<Point3d> points);
            VC_MeshType::Pointer getOutputMesh();

            // Processing
            void compute();

        private:
            PointSet<Point3d> input_;
            VC_MeshType::Pointer output_;

            //Used to add a cell to the itk mesh
            void addCell_(unsigned long a, unsigned long b, unsigned long c);
        };

    }
}
