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
    PointSet<Point3d> temp(5,5);
    temp.push_row({{0.0,0.0,0.0}, {1000.47, 53.9147, 999.983}, {839.477, 546.928, 999.983}, {669.318, 745.564, 999.983}, {453.545, 893.392, 999.983}});
    temp.push_row({{206.864, 980.336, 999.983},{-53.9147, 1000.47, 999.983}, {-311.019, 952.428, 999.983}, {-546.928, 839.477, 999.983},{-707.107, 707.107, 1000.0}});
    temp.push_row({{-826.549, 566.276, 999.983}, {-944.948, 333.054, 999.983}, {-998.951, 77.1345, 999.983}, { -984.876, -184.041, 999.983}, {-903.684, -432.675, 999.983}});
    temp.push_row({{-793.353, -608.761, 1000.0}, {-669.318, -745.564, 999.983}, {-453.545, -893.392, 999.983},{-206.864, -980.336, 999.983}, {53.9147, -1000.47, 999.983}});
    temp.push_row({{311.019, -952.428, 999.983},{546.928, -839.477, 999.983}, {745.564, -669.318, 999.983}, {893.392, -453.545, 999.983},{980.336, -206.864, 999.983}});

    volcart::meshing::OrderedPointSetMesher mesher(temp);
    mesher.compute();
    VC_MeshType::Pointer output= mesher.getOutput();

    volcart::io::objWriter writer;
    writer.setPath("OrderedPointSetMesherExample.obj");
    writer.setMesh(output);
    writer.write();

    return  EXIT_SUCCESS;

};