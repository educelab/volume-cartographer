//
// Created by Hannah Hatch on 7/25/16.
//

#ifndef VC_ORDEREDRESAMPLING_H
#define VC_ORDEREDRESAMPLING_H

#include <iostream>
#include <vector>
#include <itkMesh.h>
#include <vc_defines.h>
#include "vc_datatypes.h"


namespace volcart {
    namespace meshing {
        class orderedResampling {

        public:
            orderedResampling();
            orderedResampling(VC_MeshType::Pointer mesh, int in_width, int in_height);
            void setParamters(VC_MeshType::Pointer mesh, int in_width, int in_height);
            VC_MeshType::Pointer getOutput();
            void compute();
            void _addCell(VC_MeshType::Pointer ResampleMesh, unsigned long point1,unsigned long point2,unsigned long point3,unsigned long &cell_count);
        private:
            VC_MeshType::Pointer _input;
            VC_MeshType::Pointer _output;
            int width;
            int height;

        }; //orderedResampling
    } //meshing
} //volcart


#endif //VC_ORDEREDRESAMPLING_H
