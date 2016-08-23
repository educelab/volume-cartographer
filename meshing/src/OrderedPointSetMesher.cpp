//
// Created by Hannah Hatch on 8/23/16.
//

#include <meshing/CalculateNormals.h>
#include "meshing/OrderedPointSetMesher.h"

using namespace volcart::meshing;

OrderedPointSetMesher::OrderedPointSetMesher(){
    input_.push_back({0.0,0.0,0.0});
}

OrderedPointSetMesher::OrderedPointSetMesher(PointSet<Point3d> points){
    input_ = points;
}

VC_MeshType::Pointer OrderedPointSetMesher::getOutput(){
    return output_;
}

void OrderedPointSetMesher::setPointSet(PointSet<Point3d> points){
    input_ = points;
}

void OrderedPointSetMesher::compute(){
    VC_PointType temp_pt;
    output_ = VC_MeshType::New();
    unsigned long cnt = 0;

    for (auto i = input_.begin(); i < input_.end(); i++, cnt++)
    {
        temp_pt[0] = i[0][0];
        temp_pt[1] = i[0][1];
        temp_pt[2] = i[0][2];

        output_->SetPoint(cnt, temp_pt);

    }

    unsigned long point1, point2, point3, point4;

    for(unsigned long i = 0; i <input_.height() -1; i++){
        for(unsigned long j = 0; j <input_.width() - 1; j++){
            point1 = i *  input_.width() + j;
            point2 = point1 + 1;
            point3 = point2 + input_.width();
            point4 = point3 -1;

            if( point1 >= output_->GetNumberOfPoints() ||
                point2 >= output_->GetNumberOfPoints() ||
                point3 >= output_->GetNumberOfPoints() ||
                point4 >= output_->GetNumberOfPoints() ) {
                throw std::out_of_range("Predicted vertex index for face generation out of range of point set.");
            }

            _addCell(point2, point3, point4);
            _addCell(point1, point2, point4);
        }//j loop
    }//i loop

    volcart::meshing::CalculateNormals calcNorm(output_);
    calcNorm.compute();
    output_ = calcNorm.getMesh();
}//end func

void OrderedPointSetMesher::_addCell( unsigned long a, unsigned long b, unsigned long c )
{
    VC_CellType::CellAutoPointer current_C;

    current_C.TakeOwnership(new VC_TriangleType);

    current_C->SetPointId(0, a);
    current_C->SetPointId(1, b);
    current_C->SetPointId(2, c);

    output_->SetCell( output_->GetNumberOfCells(), current_C );
}