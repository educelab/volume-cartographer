//
// Created by Hannah Hatch on 7/25/16.
/*Algorithm takes in a base mesh and reduces the number of points and faces by
 * removing every other point along
 * horizontal and vertical axes*/
#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>

#include "core/vc_defines.h"

namespace volcart
{
namespace meshing
{
class OrderedResampling
{
public:
    OrderedResampling();
    OrderedResampling(ITKMesh::Pointer mesh, int in_width, int in_height);

    void setMesh(ITKMesh::Pointer mesh, int in_width, int in_height);
    ITKMesh::Pointer getOutputMesh() const;
    int getOutputWidth() const;
    int getOutputHeight() const;

    void compute();

private:
    ITKMesh::Pointer _input;
    int _inWidth;   // how many rows
    int _inHeight;  // how many points per row

    ITKMesh::Pointer _output;
    int _outWidth;
    int _outHeight;

    void _addCell(unsigned long a, unsigned long b, unsigned long c);

};  // OrderedResampling
}  // meshing
}  // volcart
