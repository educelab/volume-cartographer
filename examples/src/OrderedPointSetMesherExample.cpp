#include <iostream>

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/types/OrderedPointSet.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"

using namespace volcart;

auto main() -> int
{
    // Plane
    OrderedPointSet<cv::Vec3d> Plane(5);
    Plane.pushRow(
        {{0.0, 0.0, 0.0},
         {0.0, 0.0, 2.0},
         {0.0, 0.0, 4.0},
         {0.0, 0.0, 6.0},
         {0.0, 0.0, 8.0}});
    Plane.pushRow(
        {{2.0, 0.0, 0.0},
         {2.0, 0.0, 2.0},
         {2.0, 0.0, 4.0},
         {2.0, 0.0, 6.0},
         {2.0, 0.0, 8.0}});
    Plane.pushRow(
        {{4.0, 0.0, 0.0},
         {4.0, 0.0, 2.0},
         {4.0, 0.0, 4.0},
         {4.0, 0.0, 6.0},
         {4.0, 0.0, 8.0}});
    Plane.pushRow(
        {{6.0, 0.0, 0.0},
         {6.0, 0.0, 2.0},
         {6.0, 0.0, 4.0},
         {6.0, 0.0, 6.0},
         {6.0, 0.0, 8.0}});
    Plane.pushRow(
        {{8.0, 0.0, 0.0},
         {8.0, 0.0, 2.0},
         {8.0, 0.0, 4.0},
         {8.0, 0.0, 6.0},
         {8.0, 0.0, 8.0}});

    volcart::meshing::OrderedPointSetMesher mesher_plane(Plane);
    mesher_plane.compute();
    ITKMesh::Pointer output_plane = mesher_plane.getOutputMesh();

    volcart::io::OBJWriter writer;
    writer.setPath("OrderedPointSetMesher_Plane.obj");
    writer.setMesh(output_plane);
    writer.write();

    // Arch
    OrderedPointSet<cv::Vec3d> Arch(5);
    Arch.pushRow(
        {{5.0, 0.0, 0.0},
         {4.04508, 2.93893, 0.0},
         {1.54508, 4.75528, 0.0},
         {-1.54508, 4.75528, 0.0},
         {-4.04508, 2.93893, 0.0}});
    Arch.pushRow(
        {{5.0, 0.0, 2.0},
         {4.04508, 2.93893, 2.0},
         {1.54508, 4.75528, 2.0},
         {-1.54508, 4.75528, 2.0},
         {-4.04508, 2.93893, 2.0}});
    Arch.pushRow(
        {{5.0, 0.0, 4.0},
         {4.04508, 2.93893, 4.0},
         {1.54508, 4.75528, 4.0},
         {-1.54508, 4.75528, 4.0},
         {-4.04508, 2.93893, 4.0}});
    Arch.pushRow(
        {{5.0, 0.0, 6.0},
         {4.04508, 2.93893, 6.0},
         {1.54508, 4.75528, 6.0},
         {-1.54508, 4.75528, 6.0},
         {-4.04508, 2.93893, 6.0}});
    Arch.pushRow(
        {{5.0, 0.0, 8.0},
         {4.04508, 2.93893, 8.0},
         {1.54508, 4.75528, 8.0},
         {-1.54508, 4.75528, 8.0},
         {-4.04508, 2.93893, 8.0}});

    volcart::meshing::OrderedPointSetMesher mesher_arch(Arch);
    mesher_arch.compute();
    ITKMesh::Pointer output_arch = mesher_arch.getOutputMesh();

    writer.setPath("OrderedPointSetMesher_Arch.obj");
    writer.setMesh(output_arch);
    writer.write();

    return EXIT_SUCCESS;
}
