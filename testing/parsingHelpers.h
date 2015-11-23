//
// Created by Ryan Taber on 11/19/15.
//

#ifndef VC_PARSINGHELPERS_H
#define VC_PARSINGHELPERS_H

#include "vc_defines.h"
#include "shapes.h"
#include "itk2vtk.h"
#include <iostream>
#include <fstream>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>

/*
 * Purpose of File:
 *     - provides helpful parsing methods that are used by multiple
 *       testing files when reading in mesh or point cloud data
 */

namespace volcart {
    namespace testing {

        void parsePlyFile(std::string filepath, std::vector<VC_Vertex> &verts, std::vector<VC_Cell> &faces);
        void parseObjFile(std::string filepath, std::vector<VC_Vertex> &points, std::vector<VC_Cell> &cells);
        std::vector<std::string> split_string(std::string input);

    }
}
#endif //VC_PARSINGHELPERS_H
