//
// Created by Hannah Hatch on 8/23/16.
//

#include <iostream>
#include "common/io/objWriter.h"
#include "common/types/OrderedPointSet.h"
#include "common/types/Point.h"
#include "common/vc_defines.h"
#include "meshing/OrderedPointSetMesher.h"

using namespace volcart;

int main()
{

    // Plane
    OrderedPointSet<Point3d> Plane(5);
    Plane.push_row({{0.0, 0.0, 0.0},
                    {0.0, 0.0, 2.0},
                    {0.0, 0.0, 4.0},
                    {0.0, 0.0, 6.0},
                    {0.0, 0.0, 8.0}});
    Plane.push_row({{2.0, 0.0, 0.0},
                    {2.0, 0.0, 2.0},
                    {2.0, 0.0, 4.0},
                    {2.0, 0.0, 6.0},
                    {2.0, 0.0, 8.0}});
    Plane.push_row({{4.0, 0.0, 0.0},
                    {4.0, 0.0, 2.0},
                    {4.0, 0.0, 4.0},
                    {4.0, 0.0, 6.0},
                    {4.0, 0.0, 8.0}});
    Plane.push_row({{6.0, 0.0, 0.0},
                    {6.0, 0.0, 2.0},
                    {6.0, 0.0, 4.0},
                    {6.0, 0.0, 6.0},
                    {6.0, 0.0, 8.0}});
    Plane.push_row({{8.0, 0.0, 0.0},
                    {8.0, 0.0, 2.0},
                    {8.0, 0.0, 4.0},
                    {8.0, 0.0, 6.0},
                    {8.0, 0.0, 8.0}});

    volcart::meshing::OrderedPointSetMesher mesher_plane(Plane);
    mesher_plane.compute();
    VC_MeshType::Pointer output_plane = mesher_plane.getOutputMesh();

    volcart::io::objWriter writer;
    writer.setPath("OrderedPointSetMesher_Plane.obj");
    writer.setMesh(output_plane);
    writer.write();

    // Arch
    OrderedPointSet<Point3d> Arch(5);
    Arch.push_row({{5.0, 0.0, 0.0},
                   {4.04508, 2.93893, 0.0},
                   {1.54508, 4.75528, 0.0},
                   {-1.54508, 4.75528, 0.0},
                   {-4.04508, 2.93893, 0.0}});
    Arch.push_row({{5.0, 0.0, 2.0},
                   {4.04508, 2.93893, 2.0},
                   {1.54508, 4.75528, 2.0},
                   {-1.54508, 4.75528, 2.0},
                   {-4.04508, 2.93893, 2.0}});
    Arch.push_row({{5.0, 0.0, 4.0},
                   {4.04508, 2.93893, 4.0},
                   {1.54508, 4.75528, 4.0},
                   {-1.54508, 4.75528, 4.0},
                   {-4.04508, 2.93893, 4.0}});
    Arch.push_row({{5.0, 0.0, 6.0},
                   {4.04508, 2.93893, 6.0},
                   {1.54508, 4.75528, 6.0},
                   {-1.54508, 4.75528, 6.0},
                   {-4.04508, 2.93893, 6.0}});
    Arch.push_row({{5.0, 0.0, 8.0},
                   {4.04508, 2.93893, 8.0},
                   {1.54508, 4.75528, 8.0},
                   {-1.54508, 4.75528, 8.0},
                   {-4.04508, 2.93893, 8.0}});

    volcart::meshing::OrderedPointSetMesher mesher_arch(Arch);
    mesher_arch.compute();
    VC_MeshType::Pointer output_arch = mesher_arch.getOutputMesh();

    writer.setPath("OrderedPointSetMesher_Arch.obj");
    writer.setMesh(output_arch);
    writer.write();

    return EXIT_SUCCESS;
};
