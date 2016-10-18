//
// Created by Hannah Hatch on 10/18/16.
//
#include "common/io/PlyReader2.h"

typedef std::pair<char,int> props;

bool PLYReader(boost::filesystem::path path, volcart::ITKMesh::Pointer mesh)
{
    // open ply file
    std::ifstream plyFile(path.string());
    if (!plyFile.is_open()) {
        std::cerr << "Open file " << path << " failed." << std::endl;
        return false;
    }

    std::string line;
    int numofvertices;
    int numoffaces;

   
    getline(plyFile, line);

    while(line.find("element") == std::string::npos)
    {
        getline(plyFile, line);
    }

    if(line.find("vertex") == std::string::npos)
    {
        std::cerr << "No vertex information found" << std::endl;
        return false;
    }
    else
        numofvertices = line[2];

    int linecnt = 0;
    std::map<char,int> properties;
    getline(plyFile, line);

    while(line.find("element") == std::string::npos)
    {
        properties[line[2]] = linecnt;
        linecnt++;
        getline(plyFile,line);
    }

    if(line.find("faces") == std::string::npos)
    {
        numoffaces = 0;
    }
    else
        numoffaces = line[2];

    getline(plyFile,line);

    bool leadingchar = true;
    if(line.find("uchar") == std::string::npos)
        leadingchar = false;
}