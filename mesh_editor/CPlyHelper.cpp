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
	// REVISIT - quick and dirty read, specific for the file written by write_mesh() in simulation/simulation.cpp
	string aLine;

	// read header
	int aNumVertices, aNumFaces;
	const int NUM_LINES_IN_HEADER = 18;
	const int NUM_VERTEX_POS = 15;
	const int NUM_FACE_POS   = 13;
	for ( int i = 0; i < NUM_LINES_IN_HEADER; ++i ) {
		getline( aMeshFile, aLine );
		if ( aLine.find( "element vertex " ) != string::npos ) {
			aNumVertices = std::atoi( aLine.substr( NUM_VERTEX_POS ).c_str() );
		}
		if ( aLine.find( "element face " ) != string::npos ) {
			aNumFaces = std::atoi( aLine.substr( NUM_FACE_POS ).c_str() );
		}
	}
	// REVISIT - for debug
	cout << "# of vertices to be read: " << aNumVertices << endl;
	cout << "# of faces to be read: " << aNumFaces << endl;

	// read vertices
	double x, y, z, nx, ny, nz, s, t;
	int red, green, blue;
	for ( int i = 0; i < aNumVertices; ++i ) {
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
	int aTmpInt, p1, p2, p3;
	for ( int i = 0; i < aNumFaces; ++i ) {
		aMeshFile >> aTmpInt >> p1 >> p2 >> p3;
        nMesh.fFaces.push_back( Vec3< int >( p1, p2, p3 ) );
		// add edges
		// make sure edge vertices are ordered
        nMesh.fEdges.insert( p1 > p2 ? Vec2< int >( p2, p1 ) : Vec2< int >( p1, p2 ) );
        nMesh.fEdges.insert( p2 > p3 ? Vec2< int >( p3, p2 ) : Vec2< int >( p2, p3 ) );
        nMesh.fEdges.insert( p3 > p1 ? Vec2< int >( p1, p3 ) : Vec2< int >( p3, p1 ) );
		// REVISIT - for debug
	//	printf( "p1p2p3: %d %d %d edge #: %d, ", p1, p2, p3, nMesh.fEdges.size() );
	//	char x = getc(stdin);
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
