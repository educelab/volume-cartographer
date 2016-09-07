// CMesh.h
// Chao Du Sept. 2014
#pragma once

#include <iostream>
#ifndef Q_MOC_RUN
#include <pcl/point_types.h>
#endif
#include <opencv2/opencv.hpp>
#include <set>

namespace ChaoVis {

class EdgeCompareLess {

public:
	bool operator()( cv::Vec2i nE1,
					 cv::Vec2i nE2 ) const
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
	~CMesh( void );

	// REVISIT - for debugging purpose
	void Dump( void ) {
		std::cout << "# of points: " << fPoints.size() << std::endl;
		std::cout << "# of faces: " << fFaces.size() << std::endl;
		std::cout << "# of edges: " << fEdges.size() << std::endl;
	}

//private: // REVISIT - quick and dirty implementation, lack of sets and gets
public:
	// point list
	std::vector< pcl::PointXYZRGBNormal > fPoints;
	// face list
	std::vector< cv::Vec3i > fFaces;
	// edge list
	std::set< cv::Vec2i, EdgeCompareLess > fEdges;
	// dimensions
	int fWidth;
	int fHeight;

}; // class CMesh

} // namespace ChaoVis
