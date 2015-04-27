// CPlyHelper.cpp
// Chao Du Sept. 2014
#include "CPlyHelper.h"

#include <iostream>
#include <fstream>
#include <string>


namespace ChaoVis {

using namespace std;

// read ply file
bool CPlyHelper::ReadPlyFile( const string &nFileName,
								CMesh &nMesh )
{
	// open file
	ifstream aMeshFile( nFileName.c_str() );

	// REVISIT - for debug
//	cout << "File name: " << nFileName << endl;

	if ( !aMeshFile.is_open() ) {
		cerr << "Open file " << nFileName << " failed." << endl;
		return false;
	}

	// parse content
	string aLine;

    // read header
    int elementValue, aNumVertices, aNumFaces;
    string elementID;
    std::vector<int> aElements;
    std::vector<string> aElementIDs, parsed;

    getline( aMeshFile, aLine );
    // Read until we hit the end of the header
    while ( aLine.find("end_header") == string::npos ) {
        // For each "element" line in the ply header, parse that line to get the name of that
        // element and the number of that element that should be in the file
        if ( aLine.find ("element") != string::npos ) { 
            size_t lpos = 0;
            size_t pos = aLine.find(" ", lpos);
            
            while(pos != string::npos) { 
                parsed.push_back(aLine.substr(lpos, pos-lpos));
                lpos = pos+1;
                pos = aLine.find(" ",pos+1);
            }
            
            // pickup the last element
            parsed.push_back(aLine.substr(lpos, pos-lpos));
            
            // assumes element declaration in ply header == "element [elementID] [num_of_element]"
            elementID = parsed[1];
            elementValue = std::atoi(parsed[2].c_str());

            aElementIDs.push_back( elementID );
            aElements.push_back(elementValue);

            parsed.clear();
        }
        getline( aMeshFile, aLine );
    }

    for (int i = 0; i < aElements.size(); ++i) {
        cout << "Reading element: " << aElementIDs[i] << ", Number to be Read: " << aElements[i] << endl;
        for (int j = 0; j < aElements[i]; ++j) {
            // get the dimensions of the mesh
            if (aElementIDs[i] == "dimensions") {
                int width, height;

                aMeshFile >> width >> height;
                nMesh.fWidth = width;
                nMesh.fHeight = height;
            }

            // read vertices
            if (aElementIDs[i] == "vertex") {
                double x, y, z, nx, ny, nz, s, t;
                int red, green, blue;
                PointXYZRGBNormal aPoint;

                aMeshFile >> x >> y >> z >> nx >> ny >> nz >> s >> t >> red >> green >> blue;
                aPoint.x = x;
                aPoint.y = y;
                aPoint.z = z;
                aPoint.normal[ 0 ] = nx;
                aPoint.normal[ 1 ] = ny;
                aPoint.normal[ 2 ] = nz;
                aPoint.argb[ 0 ] = 255; // a
                aPoint.argb[ 1 ] = red; // r
                aPoint.argb[ 2 ] = green;   // g
                aPoint.argb[ 3 ] = blue;    // b
                nMesh.fPoints.push_back( aPoint );
            }
            
            // read faces
            if (aElementIDs[i] == "face") {
                int aTmpInt, p1, p2, p3;
                aMeshFile >> aTmpInt >> p1 >> p2 >> p3;
                nMesh.fFaces.push_back( Vec3< int >( p1, p2, p3 ) );
                // add edges
                // make sure edge vertices are ordered
                nMesh.fEdges.insert( p1 > p2 ? Vec2< int >( p2, p1 ) : Vec2< int >( p1, p2 ) );
                nMesh.fEdges.insert( p2 > p3 ? Vec2< int >( p3, p2 ) : Vec2< int >( p2, p3 ) );
                nMesh.fEdges.insert( p3 > p1 ? Vec2< int >( p1, p3 ) : Vec2< int >( p3, p1 ) );
                // REVISIT - for debug
                //  printf( "p1p2p3: %d %d %d edge #: %d, ", p1, p2, p3, nMesh.fEdges.size() );
                //  char x = getc(stdin);
            }
        }
    }

    aMeshFile.close();
    return true;
}

// write ply file
bool CPlyHelper::WritePlyFile( const std::string &nFileName,
								const CMesh &nMesh )
{
  std::ofstream meshFile;
  meshFile.open( nFileName.c_str() );
  std::cout << "creating mesh file" << std::endl;

  // write header
  meshFile << "ply" << std::endl
           << "format ascii 1.0" << std::endl
           << "comment Created by particle simulation https://github.com/viscenter/registration-toolkit" << std::endl
           << "element vertex " << nMesh.fPoints.size() << std::endl
           << "property float x" << std::endl
           << "property float y" << std::endl
           << "property float z" << std::endl
           << "property float nx" << std::endl
           << "property float ny" << std::endl
           << "property float nz" << std::endl
           << "property float s" << std::endl
           << "property float t" << std::endl
           << "property uchar red" << std::endl
           << "property uchar green" << std::endl
           << "property uchar blue" << std::endl
           << "element face " << nMesh.fFaces.size() << std::endl
           << "property list uchar int vertex_indices" << std::endl
           << "end_header" << std::endl;

  // write vertex information
  for ( size_t i = 0; i < nMesh.fPoints.size(); i++ ) {
    PointXYZRGBNormal aP = nMesh.fPoints[ i ];
	unsigned char c = ( unsigned char )( *reinterpret_cast< uint32_t* >( &( aP.rgb ) ) & 0x0000FF );
    meshFile << aP.x << " "
             << aP.y << " "
             << aP.z << " "
             << aP.normal[ 0 ] << " "
             << aP.normal[ 1 ] << " "
             << aP.normal[ 2 ] << " "
             << 0 << " "
             << 0 << " "
             << ( uint )c << " "
             << ( uint )c << " "
             << ( uint )c << std::endl;
  }

  // write face information
  for ( size_t i = 0; i < nMesh.fFaces.size(); i++ ) {
    Vec3< int > aFace = nMesh.fFaces[ i ];
    meshFile << "3 " << aFace[ 0 ] << " " << aFace[ 1 ] << " " << aFace[ 2 ] << std::endl;
  }

  meshFile.close();
	return false;
}

} // namespace ChaoVis
