//
// Created by Hannah Hatch on 7/26/16.
//

#ifndef VC_CALCULATENORMALS_H
#define VC_CALCULATENORMALS_H

#include<iostream>
#include <vc_defines.h>
#include <opencv2/opencv.hpp>
#include "vc_datatypes.h"
#include "deepCopy.h"


namespace volcart{
    namespace meshing{
        class calculateNormals{
        public:
            calculateNormals();
            calculateNormals(VC_MeshType::Pointer mesh);
            void setMesh(VC_MeshType::Pointer mesh);
            void computeNormals();
            VC_MeshType::Pointer getOutput();
        private:
            VC_MeshType::Pointer _in_mesh;
            VC_MeshType::Pointer _out_mesh;
            std::vector<cv::Vec3d> _vertex_normals;
        };
    }//meshing
}//volcart
#endif //VC_CALCULATENORMALS_H
