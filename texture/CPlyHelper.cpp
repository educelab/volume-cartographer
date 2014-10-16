// CPlyHelper.cpp
// Chao Du Sept. 2014
#include "CPlyHelper.h"

#include <iostream>
#include <fstream>
#include <string>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>


namespace ChaoVis {

using namespace boost;
using namespace pcl;
using namespace std;

// read ply file
bool CPlyHelper::ReadPlyFile( const string &nFileName,
								CMesh &nMesh )
{
	// open file
	ifstream aMeshFile;
	aMeshFile.open( nFileName.c_str(), ifstream::in );

	if ( !aMeshFile.is_open() ) {
		cerr << "Open file " << nFileName << " failed." << endl;
		return false;
	}

	// parse content
	// REVISIT - quick and dirty read, specific for the file written by write_mesh() in simulation/simulation.cpp
	string aLine;

	// read header
	int aNumVertices, aNumFaces;
	for ( int i = 0; i < 18; ++i ) {
		getline( aMeshFile, aLine );
		if ( aLine.find( "element vertex " ) != string::npos ) {
			aNumVertices = lexical_cast< int >( aLine.substr( 15 ) );
		}
		if ( aLine.find( "element face " ) != string::npos ) {
			aNumFaces = lexical_cast< int >( aLine.substr( 13 ) );
		}
	}
	// REVISIT - for debug
	cout << "# of vertices to be read: " << aNumVertices << endl;
	cout << "# of faces to be read: " << aNumFaces << endl;

	// read body
	vector< string > aStrings;
	while ( getline( aMeshFile, aLine ) ) {
		split( aStrings, aLine, is_any_of( "\t " ) );
		if ( aStrings.size() == 11 ) {			// vertex
			PointXYZRGBNormal aPoint;
			aPoint.x = lexical_cast< float >( aStrings[ 0 ] );
			aPoint.y = lexical_cast< float >( aStrings[ 1 ] );
			aPoint.z = lexical_cast< float >( aStrings[ 2 ] );
			aPoint.normal[ 0 ] = lexical_cast< float >( aStrings[ 3 ] );
			aPoint.normal[ 1 ] = lexical_cast< float >( aStrings[ 4 ] );
			aPoint.normal[ 2 ] = lexical_cast< float >( aStrings[ 5 ] );
			aPoint.r = lexical_cast< float >( aStrings[ 8 ] );
			aPoint.g = lexical_cast< float >( aStrings[ 9 ] );
			aPoint.b = lexical_cast< float >( aStrings[ 10 ] );
			nMesh.fPoints.push_back( aPoint );
		} else if ( aStrings.size() == 4 ) {	// face
			nMesh.fFaces.push_back( cv::Vec3i( lexical_cast< int >( aStrings[ 1 ] ),
											   lexical_cast< int >( aStrings[ 2 ] ),
											   lexical_cast< int >( aStrings[ 3 ] ) ) );
			// add edges
			cv::Vec2i aEdge( nMesh.fFaces.back()[ 0 ], nMesh.fFaces.back()[ 1 ] );
			// make sure edge vertices are ordered
			if ( aEdge[ 0 ] > aEdge[ 1 ] ) {
				aEdge = cv::Vec2i( aEdge[ 1 ], aEdge[ 0 ] );
			}
			nMesh.fEdges.insert( aEdge );
			aEdge = cv::Vec2i( nMesh.fFaces.back()[ 1 ], nMesh.fFaces.back()[ 2 ] );
			if ( aEdge[ 0 ] > aEdge[ 1 ] ) {
				aEdge = cv::Vec2i( aEdge[ 1 ], aEdge[ 0 ] );
			}
			nMesh.fEdges.insert( aEdge );
			aEdge = cv::Vec2i( nMesh.fFaces.back()[ 2 ], nMesh.fFaces.back()[ 0 ] );
			if ( aEdge[ 0 ] > aEdge[ 1 ] ) {
				aEdge = cv::Vec2i( aEdge[ 1 ], aEdge[ 0 ] );
			}
			nMesh.fEdges.insert( aEdge );
		} else {
			std::cerr << "Invalide mesh data." << std::endl;
			return false;
		}
	}

	aMeshFile.close();
	return true;
}

// write ply file
bool CPlyHelper::WritePlyFile( const std::string &nFileName,
								const CMesh &nMesh )
{
	return false;
}

} // namespace ChaoVis
