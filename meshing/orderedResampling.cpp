//
// Created by Hannah Hatch on 7/25/16.
//

#include "orderedResampling.h"

namespace volcart{
    namespace meshing{
        orderedResampling::orderedResampling()
        {
            _input = VC_MeshType::New();
            _output = NULL;
            width = 0;
            height = 0;
        }//constructor

        orderedResampling::orderedResampling(VC_MeshType::Pointer mesh, int in_width, int in_height)
        {
            _input = mesh;
            _output = NULL;
            width = in_width ;
            height = in_height;
        }//constructor with parameters

        void orderedResampling:: setParamters(VC_MeshType::Pointer mesh, int in_width, int in_height)
        {
            _input = mesh;
            width = in_width;
            height = in_height;
        }//setParameters

        VC_MeshType::Pointer  orderedResampling::getOutput()
        {
            if(_output.IsNull())
            {
                std::cerr << "Error: Output Mesh is not set" << std::endl;
                return NULL;
            }
            else
                return _output;
        }//getOutput

        void orderedResampling:: compute()
        {

            std::vector<VC_PointType> ResamplePoints;
            _output = VC_MeshType::New();

            unsigned long point1, point2, point3, point4;
            unsigned long cell_count = 0;
            unsigned long point_count = 0;
            unsigned long line_cnt = 0;

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
                    if(line_cnt != width)
                        line_cnt++;
                    else
                        line_skip = true;
                }
                //Allows every other line to be skipped
                if(line_skip && line_cnt != width)
                {
                    line_cnt++;
                }
                else if(line_skip && line_cnt == width)
                {
                    line_cnt = 0;
                    line_skip = false;
                }
                pointsIterator++;
                k++;
            }

            int new_width = (width +1)/2;

            //Create new faces based on new set of points and keeps normals same as original
            for(unsigned long i = 0; i < height; i++)
            {
                for (unsigned long j = 0; j < new_width -1 ; j++) {

                    //4 points allows us to create the upper and lower faces at the same time
                    point1 = i * new_width + j;
                    point2 = point1 + 1;
                    point3 = point2 + new_width;
                    point4 = point3 - 1;

                    if (point1 >= ResamplePoints.size() || point2 >= ResamplePoints.size() || point3 >= ResamplePoints.size()|| point4 >= ResamplePoints.size()) {
                        continue;
                    }

                    //Add both these faces to the mesh
                    _addCell(_output, point2, point3, point4, cell_count);
                    _addCell(_output, point1, point2, point4, cell_count);

                }
            }

            std::cout << std::endl << "The number of points in the resampled mesh is "<< _output->GetNumberOfPoints() << std::endl;
            std::cout << "The number of cells in the resampled mesh is "<< _output->GetNumberOfCells() << std::endl;


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