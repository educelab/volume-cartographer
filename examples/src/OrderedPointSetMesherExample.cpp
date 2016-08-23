//
// Created by Hannah Hatch on 8/23/16.
//

#include <iostream>
#include "common/vc_defines.h"
#include "common/types/Point.h"
#include "common/types/PointSet.h"
#include "common/io/objWriter.h"
#include "meshing/OrderedPointSetMesher.h"

using namespace volcart;

int main() {
    PointSet<Point3d> Plane(5,5);
    Plane.push_row({{0.0, 0.0, 0.0},{0.0, 0.0, 2.0}, {0.0, 0.0, 4.0}, {0.0, 0.0, 6.0}, {0.0, 0.0, 8.0}});
    Plane.push_row({{2.0, 0.0, 0.0},{2.0, 0.0, 2.0}, {2.0, 0.0, 4.0}, {2.0, 0.0, 6.0}, {2.0, 0.0, 8.0}});
    Plane.push_row({{4.0, 0.0, 0.0},{4.0, 0.0, 2.0}, {4.0, 0.0, 4.0}, {4.0, 0.0, 6.0}, {4.0, 0.0, 8.0}});
    Plane.push_row({{6.0, 0.0, 0.0},{6.0, 0.0, 2.0}, {6.0, 0.0, 4.0}, {6.0, 0.0, 6.0}, {6.0, 0.0, 8.0}});
    Plane.push_row({{8.0, 0.0, 0.0},{8.0, 0.0, 2.0}, {8.0, 0.0, 4.0}, {8.0, 0.0, 6.0}, {8.0, 0.0, 8.0}});


    volcart::meshing::OrderedPointSetMesher mesher_plane(Plane);
    mesher_plane.compute();
    VC_MeshType::Pointer output_plane= mesher_plane.getOutput();

    volcart::io::objWriter writer;
    writer.setPath("OrderedPointSetMesher_Plane.obj");
    writer.setMesh(output_plane);
    writer.write();

    PointSet<Point3d> Arch(7,7);
    Arch.push_row({{5.0, 0.0, 0.0},{4.69846, 1.7101, 0.0}, {3.83022, 3.21394, 0.0}, {2.5, 4.33013, 0.0}, {0.868241, 4.92404, 0.0},{-0.868241, 4.92404, 0.0},{-2.5, 4.33013, 0.0}});
    Arch.push_row({{-3.83022, 3.21394, 0.0},{-4.69846, 1.7101, 0.0}, {5.0, 0.0, 1.0}, {4.69846, 1.7101, 1.0}, {3.83022, 3.21394, 1.0},{-4.69846, 1.7101, 1.0},{5.0, 0.0, 2.0}});
    Arch.push_row({{4.69846, 1.7101, 5.0},{3.83022, 3.21394, 3.625}, {2.5, 4.33013, 0.0}, {-4.69846, 1.7101, 2.0}, {5.0, 0.0, 3.0},{-0.868241, 4.92404, 1.75},{-2.5, 4.33013, 1.75}});
    Arch.push_row({{-3.83022, 3.21394, 1.75},{-4.69846, 1.7101, 3.0}, {5.0, 0.0, 4.0}, {-4.69846, 1.7101, 4.0}, {5.0, 0.0, 5.0},{2.5, 4.33013, 5.0},{-0.868241, 4.92404, 4.5}});
    Arch.push_row({{-2.5, 4.33013, 5.5},{-3.83022, 3.21394, 4.5}, {-4.69846, 1.7101, 5.0}, {5.0, 0.0, 6.0}, {-4.69846, 1.7101, 6.0},{5.0, 0.0, 7.0},{3.83022, 3.21394, 7.0}});
    Arch.push_row({{2.5, 4.33013, 0.0},{0.868241, 4.92404, 4.4375}, {-0.868241, 4.92404, 6.5}, {-3.83022, 3.21394, 6.5}, { -4.69846, 1.7101, 7.0},{5.0, 0.0, 8.0},{4.69846, 1.7101, 8.0}});
    Arch.push_row({{3.83022, 3.21394, 8.0},{2.5, 4.33013, 8.0}, {0.868241, 4.92404, 8.0}, {-0.868241, 4.92404, 8.0}, {-2.5, 4.33013, 8.0},{-3.83022, 3.21394, 8.0},{-4.69846, 1.7101, 8.0}});

    volcart::meshing::OrderedPointSetMesher mesher_arch(Arch);
    mesher_arch.compute();
    VC_MeshType::Pointer output_arch= mesher_arch.getOutput();

    writer.setPath("OrderedPointSetMesher_Arch.obj");
    writer.setMesh(output_arch);
    writer.write();


    return  EXIT_SUCCESS;

};