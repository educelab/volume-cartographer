//
// Created by Hannah Hatch on 8/23/16.
//

#pragma once

#include "common/types/OrderedPointSet.h"
#include "common/types/Point.h"
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
    OrderedPointSetMesher(OrderedPointSet<Point3d> points) : input_(points) {}

    // I/O
    void setPointSet(OrderedPointSet<Point3d> points) { input_ = points; }
    MeshType::Pointer getOutputMesh() const { return output_; }

    // Processing
    void compute();

private:
    OrderedPointSet<Point3d> input_;
    MeshType::Pointer output_;

    // Used to add a cell to the itk mesh
    void addCell_(size_t a, size_t b, size_t c);
};

}  // meshing
}  // volcart
