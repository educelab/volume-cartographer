//
// Created by Hannah Hatch on 7/26/16.
//

#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

#include "common/vc_defines.h"

namespace volcart{
    namespace meshing{
        class CalculateNormals{
        public:
            // Construction //
            CalculateNormals();
            CalculateNormals(MeshType::Pointer mesh) ;

            // Input/Output //
            void setMesh(MeshType::Pointer mesh);
            MeshType::Pointer getMesh() const;

            // Processing //
            void compute();

        private:
            void _computeNormals();
            void _assignToMesh();

            MeshType::Pointer  _input;
            MeshType::Pointer _output;

            std::vector<cv::Vec3d> _vertex_normals; // convenience vector to store calculated normals by p_id
        };
    }//meshing
}//volcart
