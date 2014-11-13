// pathGen.cpp
// Chao Du Oct 2014
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "volumepkg.h"
#include "timeUtils.h"

using namespace std;
using namespace cv;


// note: this program generate Bezier curve from user clicks
//       It will draw curve after 3 clicks, the start and end poitns are anchor.
//       Then use the last end point as the new start point.

#define SAMPLE_RATE 1//0.5

typedef struct pt_tuple_tag {
	Vec2f start;
	Vec2f middle;
	Vec2f end;
} pt_tuple;

int					gTupleIndex;
pt_tuple			gTmpTuple;
vector< pt_tuple >	gPath;
vector< Vec2f >		gInterPath;

Mat					gImg;		// the volume slice
Mat					gImgCache;
int					gZ;			// the slice index on which the path is drawn
string				gVolPkgName;


// Bezier point
float getPt( int n1, int n2, float perc )
{
	int diff = n2 - n1;
	return n1 + ( diff * perc );
}

// draw Bezier curve
void drawBezier( Vec2f &start, Vec2f &middle, Vec2f &end )
{
	float aTotalLen = sqrt( ( middle[ 0 ] - start[ 0 ] ) * ( middle[ 0 ] - start[ 0 ] ) + 
							( middle[ 1 ] - start[ 1 ] ) * ( middle[ 1 ] - start[ 1 ] ) ) + 
					sqrt( ( middle[ 0 ] - end[ 0 ] ) * ( middle[ 0 ] - end[ 0 ] ) + 
							( middle[ 1 ] - end[ 1 ] ) * ( middle[ 1 ] - end[ 1 ] ) );
	int aNumOfPts = aTotalLen / SAMPLE_RATE;
	float aInterval = 1.0 / aNumOfPts;

	for ( int i = 0; i < aNumOfPts; i++ ) {
		float xa = getPt( start[ 0 ], middle[ 0 ], i * aInterval );
		float ya = getPt( start[ 1 ], middle[ 1 ], i * aInterval );
		float xb = getPt( middle[ 0 ], end[ 0 ], i * aInterval );
		float yb = getPt( middle[ 1 ], end[ 1 ], i * aInterval );

		float x = getPt( xa, xb, i * aInterval );
		float y = getPt( ya, yb, i * aInterval );

		circle( gImg, Point2f( x, y ), 1, Scalar( 0, 0, 65535 ) );
	}
}

void savePath( void )
{
	ofstream aOut;
	aOut.open( gVolPkgName + "_" + currentDateTime() + ".txt", ofstream::out );

	if ( !aOut.is_open() ) {
		std::cerr << "Open file " << "path.txt" << " failed" << endl;
		return;
	}

	for ( size_t i = 0; i < gPath.size(); ++i ) {
		Vec2f p1 = gPath[ i ].start;
		Vec2f p2 = gPath[ i ].middle;
		Vec2f p3 = gPath[ i ].end;
		float aTotalLen = sqrt( ( p2[ 0 ] - p1[ 0 ] ) * ( p2[ 0 ] - p1[ 0 ] ) + 
								( p2[ 1 ] - p1[ 1 ] ) * ( p2[ 1 ] - p1[ 1 ] ) ) + 
						sqrt( ( p2[ 0 ] - p3[ 0 ] ) * ( p2[ 0 ] - p3[ 0 ] ) + 
								( p2[ 1 ] - p3[ 1 ] ) * ( p2[ 1 ] - p3[ 1 ] ) );
		int aNumOfPts = aTotalLen / SAMPLE_RATE;
		float aInterval = 1.0 / aNumOfPts;
    
		for ( int i = 0; i < aNumOfPts; i++ ) {
			float xa = getPt( p1[ 0 ], p2[ 0 ], i * aInterval );
			float ya = getPt( p1[ 1 ], p2[ 1 ], i * aInterval );
			float xb = getPt( p2[ 0 ], p3[ 0 ], i * aInterval );
			float yb = getPt( p2[ 1 ], p3[ 1 ], i * aInterval );
    
			float x = getPt( xa, xb, i * aInterval );
			float y = getPt( ya, yb, i * aInterval );

			// REVISIT - Chao 20141103 - new path format: x y z x y z...
			//           x y z = slice index, image column, image row
			aOut << gZ << " " << x << " " << y << endl;
		}
	}

	aOut.close();
}

// update the view
void updateView( void )
{
	gImgCache.copyTo( gImg );

	for ( size_t i = 0; i < gPath.size(); ++i ) {
		drawBezier( gPath[ i ].start, gPath[ i ].middle, gPath[ i ].end );
	}

	gImg.copyTo( gImgCache );

	if ( gTupleIndex == 2 ) {
		drawBezier( gTmpTuple.start, gTmpTuple.middle, gTmpTuple.end );
	}
	// update display
	imshow( "pathGen", gImg );
}

// mouse click callback function
void MouseClickCallBackFunc( int e, int x, int y, int flags, void *userdata )
{
	if ( e == EVENT_LBUTTONDOWN ) {
		// REVISIT - fill me here
//		cout << "x: " << x << ", y: " << y << endl;
		if ( gTupleIndex == 0 ) {
			gTmpTuple.start[ 0 ] = x;
			gTmpTuple.start[ 1 ] = y;
		} else if ( gTupleIndex == 1 ) {
			gTmpTuple.middle[ 0 ] = x;
			gTmpTuple.middle[ 1 ] = y;
		} else if ( gTupleIndex == 2 ) {
			gTmpTuple.end[ 0 ] = x;
			gTmpTuple.end[ 1 ] = y;

			gPath.push_back( gTmpTuple );
			gTupleIndex = ( gTupleIndex + 1 ) % 3;

			// start a new segment with the last end as the new start
			gTmpTuple.start[ 0 ] = x;
			gTmpTuple.start[ 1 ] = y;
		}
		gTupleIndex = ( gTupleIndex + 1 ) % 3;

//		cout << "Index: " << gTupleIndex << endl;
	} else if ( e == EVENT_LBUTTONUP ) {
		// REVISIT - fill me here
	} else if ( e == EVENT_MOUSEMOVE ) {
		if ( gTupleIndex == 2 ) {
			gTmpTuple.end[ 0 ] = x;
			gTmpTuple.end[ 1 ] = y;
		}
	} else if ( e == EVENT_RBUTTONDOWN ) {
		// REVISIT - fill me here
		cout << "Save path to file and exit" << endl;
		savePath();
		exit( 0 );
	} else if ( e == EVENT_MBUTTONDOWN ) {
	}

	updateView();
}

int main( int argc, char *argv[] )
{
	if ( argc != 3 ) {
		cout << " Insufficient arguments. Usage: pathGen volumePkgPath sliceIndex " << endl;
		return -1;
	}

	gZ = atoi( argv[ 2 ] );

	// REVISIT - should VolumePkg be singleton?
	VolumePkg vpkg = VolumePkg( argv[ 1 ] );
	gVolPkgName = vpkg.getPkgName();

	if ( gZ < 3 || gZ > vpkg.getNumberOfSlices() - 2 ) {
		cout << "ERROR: slice index out of range. Please select a number between " << 3 << " and " << vpkg.getNumberOfSlices() - 2 << endl;
		return -2;
	}

	vpkg.getSliceAtIndex( gZ ).copyTo ( gImg );
	cvtColor( gImg, gImg, CV_GRAY2BGR );
	gImg.copyTo( gImgCache );

	if ( !gImg.data ) {
		cout << " Error. Image is not loaded properly. " << endl;
		return -2;
	}

	gTupleIndex = 0;

	// create a window
	namedWindow( "pathGen", WINDOW_AUTOSIZE );

	// set the callback fucntion for any mosue event
	setMouseCallback( "pathGen", MouseClickCallBackFunc, NULL );

	// show image on the window
	imshow( "pathGen", gImg );

	waitKey( 0 );
	return 0;
}
