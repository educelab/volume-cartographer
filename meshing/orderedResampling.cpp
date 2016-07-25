//
// Created by Hannah Hatch on 7/25/16.
//

#include "orderedResampling.h"

namespace volcart{
    namespace meshing{
        orderedResampling::orderedResampling()
        {
            _input = nullptr;
            _output = nullptr;
            _width = 0;
            _height = 0;
        }//constructor

        orderedResampling::orderedResampling(VC_MeshType::Pointer mesh, int in_width, int in_height)
        {
            _input = mesh;
            _output = nullptr;
           _width = in_width ;
            _height = in_height;
        }//constructor with parameters

        void orderedResampling::setMesh(VC_MeshType::Pointer mesh, int in_width, int in_height)
        {
            _input = mesh;
            _width = in_width;
            _height = in_height;
        }//setParameters

        VC_MeshType::Pointer orderedResampling::getOutput()
        {
            if(_output.IsNull())
            {
                std::cerr << "Error: Output Mesh is not set" << std::endl;
                return NULL;
            }
            else
                return _output;
        }//getOutput

        void orderedResampling::compute()
        {

            std::vector<VC_PointType> ResamplePoints;
            _output = VC_MeshType::New();

            //Vertices for each face in the new mesh
            unsigned long point1, point2, point3, point4;

            unsigned long cell_count = 0;
            unsigned long point_count = 0;
            unsigned long line_cnt = 0;

            //Tells the loop whether or not the points in that line should be added to the new mesh
            bool line_skip = false;

            //Loop iterator
            unsigned long k =0;
            VC_PointsInMeshIterator pointsIterator = _input->GetPoints()->Begin();

            //Adds certain points from old mesh into the new mesh
            while(pointsIterator != _input->GetPoints()->End())
            {
                if(!line_skip)
                {
                    //Keeps every other point in the row
                    if(k % 2 == 0)
                    {
                        ResamplePoints.push_back(pointsIterator.Value());
                        _output->SetPoint(point_count, pointsIterator.Value());
                        point_count++;
                    }
                    if(line_cnt != _width)
                        line_cnt++;
                    else
                        line_skip = true;
                }
                //Allows every other line to be skipped
                if(line_skip && line_cnt != _width)
                {
                    line_cnt++;
                }
                else if(line_skip && line_cnt == _width)
                {
                    line_cnt = 0;
                    line_skip = false;
                }
                pointsIterator++;
                k++;
            }
            //New width of mesh, no need to do this for height as it will shrink the mesh too much
            int new_width = _width / 2;
            int new_height = (_height);

            //Create two new faces each iteration based on new set of points and keeps normals same as original
            for(unsigned long i = 0; i < new_height -1; i++)
            {
                for (unsigned long j = 0; j < new_width - 1; j++) {

                    //4 points allows us to create the upper and lower faces at the same time
                    point1 = i * new_width + j;
                    point2 = point1 + 1;
                    point3 = point2 + new_width;
                    point4 = point3 - 1;

                    try {
                        std::out_of_range oor("Out of range");
                        if(point1 >= ResamplePoints.size() || point2 >= ResamplePoints.size() || point3 >= ResamplePoints.size() ||point4 >= ResamplePoints.size())
                            throw oor;
                    }
                    catch(const std::out_of_range& oor)
                    {
                        std::cerr << "Error: Out of range, skipping face" << std::endl;
                        continue;
                    }


                    //Add both these faces to the mesh
                    _addCell(_output, point2, point3, point4, cell_count);
                    _addCell(_output, point1, point2, point4, cell_count);

                }
            }

            std::cerr << "volcart::meshing::orderedResampling: Points in resampled mesh "<< _output->GetNumberOfPoints() << std::endl;
            std::cerr << "volcart::meshing::orderedResampling: Cells in resampled mesh "<< _output->GetNumberOfCells() << std::endl;


        }//compute

        void orderedResampling::_addCell(VC_MeshType::Pointer ResampleMesh, unsigned long point1,unsigned long point2,unsigned long point3,unsigned long &cell_count)
        {
            VC_CellType::CellAutoPointer current_C;

            current_C.TakeOwnership(new VC_TriangleType);

            current_C->SetPointId(0, point1);
            current_C->SetPointId(1, point2);
            current_C->SetPointId(2, point3);

            ResampleMesh->SetCell(cell_count, current_C);
            cell_count++;
        }//addCell

    }//meshing
}//volcart