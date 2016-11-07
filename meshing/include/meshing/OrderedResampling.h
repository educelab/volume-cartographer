//
// Created by Hannah Hatch on 7/25/16.
/*Algorithm takes in a base mesh and reduces the number of points and faces by
 * removing every other point along
 * horizontal and vertical axes*/
#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>

#include "common/vc_defines.h"

namespace volcart
{
namespace meshing
{
class OrderedResampling
{
public:
    OrderedResampling();
    OrderedResampling(
        ITKMesh::Pointer mesh, uint32_t in_width, uint32_t in_height);

    void setMesh(ITKMesh::Pointer mesh, uint32_t in_width, uint32_t in_height);
    ITKMesh::Pointer getOutputMesh() const;
    uint32_t getOutputWidth() const;
    uint32_t getOutputHeight() const;

    void compute();

private:
    ITKMesh::Pointer _input;
    uint32_t _inWidth;   // how many rows
    uint32_t _inHeight;  // how many points per row

    ITKMesh::Pointer _output;
    uint32_t _outWidth;
    uint32_t _outHeight;

    void _addCell(uint32_t a, uint32_t b, uint32_t c);

};  // OrderedResampling
}  // meshing
}  // volcart
