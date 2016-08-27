//
// Created by Hannah Hatch on 8/23/16.
//

#pragma once

#include "common/types/Point.h"
#include "common/types/PointSet.h"
#include "common/vc_defines.h"

namespace volcart
{
namespace meshing
{

class OrderedPointSetMesher
{
public:
    // Initalizers
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

    // Used to add a cell to the itk mesh
    void addCell_(size_t a, size_t b, size_t c);
};

}  // meshing
}  // volcart
