//
// Created by Seth Parker on 4/27/15.
//
#include <iostream>
#include <fstream>
#include <cmath>

#include <itkMesh.h>
#include <itkTriangleCell.h>

#ifndef VC_PLYHELPER_H
#define VC_PLYHELPER_H

typedef itk::Vector< double, 3 >    PixelType;          // A vector to hold the normals along with the points of each vertice in the mesh
const unsigned int Dimension = 3;                       // Need a 3 Dimensional Mesh
typedef itk::Mesh< PixelType, Dimension >   MeshType;   // declare Mesh object using template parameters

typedef MeshType::CellType                CellType;
typedef itk::TriangleCell< CellType >     TriangleType;

bool ply2itkmesh (std::string plyPath,
                  itk::Mesh<::itk::Vector<double, 3>, 3>::Pointer mesh,
                  int &width,
                  int &height);

#endif //VC_PLYHELPER_H
