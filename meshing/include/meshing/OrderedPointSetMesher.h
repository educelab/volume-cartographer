//
// Created by Hannah Hatch on 8/23/16.
//

#pragma once

#include <iostream>
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


            VC_MeshType::Pointer getOutput();
            void setPointSet(PointSet<Point3d> points);
            void compute();

        private:
            PointSet<Point3d> input_;
            VC_MeshType::Pointer output_ = nullptr;

            //Used to add a cell to the itk mesh
            void addCell_(unsigned long a, unsigned long b, unsigned long c);
        };

    }
}
