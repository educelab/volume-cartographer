// CMesh.h
// Chao Du Sept. 2014
#ifndef _CMESH_H_
#define _CMESH_H_

#include <iostream>
#include "CPoint.h"
#include <map>
#include <set>
#include <vector>

#include "mathUtils.h"

namespace ChaoVis {

class EdgeCompareLess {

public:
	bool operator()( const Vec2< int > &nE1,
					 const Vec2< int > &nE2 ) const
	{
		if ( nE1[ 0 ] < nE2[ 0 ] ) {
			return true;
		} else if ( nE1[ 0 ] == nE2[ 0 ] ) {
			return ( nE1[ 1 ] < nE2[ 1 ] );
		} else {
			return false;
		}
	}

}; // class EdgeCompareLess

class CMesh {

public:
	CMesh( void );
    virtual ~CMesh( void );

	// REVISIT - for debugging purpose
	void Dump( void ) {
		std::cout << "# of points: " << fPoints.size() << std::endl;
		std::cout << "# of faces: " << fFaces.size() << std::endl;
		std::cout << "# of edges: " << fEdges.size() << std::endl;
	}

//private: // REVISIT - quick and dirty implementation, lack of sets and gets
public:
	// point list
	std::vector< ChaoVis::PointXYZRGBNormal > fPoints;
	// face list
	std::vector< Vec3< int > > fFaces;
	// edge list
	std::set< Vec2< int >, EdgeCompareLess > fEdges;

}; // class CMesh

} // namespace ChaoVis

#endif // _CMESH_H_
