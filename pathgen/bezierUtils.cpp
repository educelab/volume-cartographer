// bezierUtils.cpp
// Chao Du 2014 Nov
#include "bezierUtils.h"

#include <fstream>


using namespace std;
using namespace cv;

// Bezier point
float GetPt( int n1, 
			int n2, 
			float percent )
{
	int diff = n2 - n1;
	return n1 + ( diff * percent );
}

// draw Bezier curve
void DrawBezier( cv::Vec2f &start, 
				cv::Vec2f &middle, 
				cv::Vec2f &end,
				cv::Mat &nImg )
{
	float aTotalLen = sqrt( ( middle[ 0 ] - start[ 0 ] ) * ( middle[ 0 ] - start[ 0 ] ) + 
							( middle[ 1 ] - start[ 1 ] ) * ( middle[ 1 ] - start[ 1 ] ) ) + 
					sqrt( ( middle[ 0 ] - end[ 0 ] ) * ( middle[ 0 ] - end[ 0 ] ) + 
							( middle[ 1 ] - end[ 1 ] ) * ( middle[ 1 ] - end[ 1 ] ) );
	int aNumOfPts = aTotalLen / CURVE_SAMPLE_RATE;
	float aInterval = 1.0 / aNumOfPts;

	for ( int i = 0; i < aNumOfPts; i++ ) {
		float xa = GetPt( start[ 0 ], middle[ 0 ], i * aInterval );
		float ya = GetPt( start[ 1 ], middle[ 1 ], i * aInterval );
		float xb = GetPt( middle[ 0 ], end[ 0 ], i * aInterval );
		float yb = GetPt( middle[ 1 ], end[ 1 ], i * aInterval );

		float x = GetPt( xa, xb, i * aInterval );
		float y = GetPt( ya, yb, i * aInterval );

		circle( nImg, cv::Point2f( x, y ), 1, cv::Scalar( 0, 0, 65535 ) );
	}
}

bool SavePath( VolumePkg *vpkg,
				const std::vector< pt_tuple > &nPath,
				int nPathOnSliceIndex )
{
	pcl::PointCloud<pcl::PointXYZRGB> pathCloud;

	for ( size_t i = 0; i < nPath.size(); ++i ) {
		Vec2f p1 = nPath[ i ].start;
		Vec2f p2 = nPath[ i ].middle;
		Vec2f p3 = nPath[ i ].end;
		float aTotalLen = sqrt( ( p2[ 0 ] - p1[ 0 ] ) * ( p2[ 0 ] - p1[ 0 ] ) + 
								( p2[ 1 ] - p1[ 1 ] ) * ( p2[ 1 ] - p1[ 1 ] ) ) + 
						sqrt( ( p2[ 0 ] - p3[ 0 ] ) * ( p2[ 0 ] - p3[ 0 ] ) + 
								( p2[ 1 ] - p3[ 1 ] ) * ( p2[ 1 ] - p3[ 1 ] ) );
		int aNumOfPts = aTotalLen / CURVE_SAMPLE_RATE;
		float aInterval = 1.0 / aNumOfPts;
    
		for ( int i = 0; i < aNumOfPts; i++ ) {
			float xa = GetPt( p1[ 0 ], p2[ 0 ], i * aInterval );
			float ya = GetPt( p1[ 1 ], p2[ 1 ], i * aInterval );
			float xb = GetPt( p2[ 0 ], p3[ 0 ], i * aInterval );
			float yb = GetPt( p2[ 1 ], p3[ 1 ], i * aInterval );
    
			float x = GetPt( xa, xb, i * aInterval );
			float y = GetPt( ya, yb, i * aInterval );

			// REVISIT - Chao 20141103 - new path format: x y z x y z...
			//           x y z = slice index, image column, image row
			pcl::PointXYZRGB point;
      		point.x = nPathOnSliceIndex;
      		point.y = x;
      		point.z = y;
			pathCloud.push_back(point);
		}
	}

	vpkg->setActiveSegmentation(vpkg->newSegmentation());
	vpkg->saveCloud(pathCloud);

	return true;
}
