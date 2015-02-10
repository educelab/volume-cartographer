// UDataManipulateUtils.cpp
// Chao Du 2014 Dec
#include "HBase.h"
#include "UDataManipulateUtils.h"


namespace ChaoVis {

// Split vertex and face to memory chunk that can fit into OpenGL
bool SplitVertexAndElementBuffer( int				nVertexNum,
                                  int				nFaceNum,
                                  const int			*nElementBufferTmp,	// constant data, not constant pointer
                                  unsigned short	***nElementBufferData,
                                  const float		*nVertexBufferTmp,
                                  float				***nVertexBufferData,
                                  const float		*nUVBufferTmp,
                                  float				***nUVBufferData,
                                  int				**nElementBufferSize,
                                  int				**nVertexBufferSize,
                                  int				**nUVBufferSize,
                                  int				*nElementArrayNum )
{
    const unsigned short MAX_NUM_FACE_IN_ARRAY = USHORT_SIZE / 3;
    const unsigned short MAX_NUM_VERTEX_IN_ARRAY = MAX_NUM_FACE_IN_ARRAY * 3;

    int aArraySize = ( nFaceNum * 3 / USHORT_SIZE ) + 1;
    *nElementArrayNum = aArraySize;

    *nElementBufferData = new unsigned short*[ aArraySize ];
    *nElementBufferSize = new int            [ aArraySize ];

    *nVertexBufferData  = new float*[ aArraySize ];
    *nVertexBufferSize  = new int   [ aArraySize ];

    *nUVBufferData      = new float*[ aArraySize ];
    *nUVBufferSize      = new int   [ aArraySize ];

    // iterate through every face (triangle), which is composed of 3 vertices, and distribute to different arrays
    size_t aSurfaceBase = 0;
    size_t aArrayIndex = 0;

    while ( aSurfaceBase < nFaceNum ) {

        size_t aSurfaceCntRemained = nFaceNum - aSurfaceBase;
        size_t aSurfaceCntToProcess = MAX_NUM_FACE_IN_ARRAY < aSurfaceCntRemained ? MAX_NUM_FACE_IN_ARRAY : aSurfaceCntRemained;

        ( *nElementBufferData )[ aArrayIndex ] = new unsigned short[ aSurfaceCntToProcess * 3     ];
        ( *nVertexBufferData  )[ aArrayIndex ] = new float         [ aSurfaceCntToProcess * 3 * 4 ];	// REVISIT - 4 for homogeneous coordinates
        ( *nUVBufferData      )[ aArrayIndex ] = new float         [ aSurfaceCntToProcess * 3 * 2 ];

//		int aIndexOffset = aSurfaceBase * 3;
        ( *nElementBufferSize )[ aArrayIndex ] = aSurfaceCntToProcess * 3;
        ( *nVertexBufferSize  )[ aArrayIndex ] = aSurfaceCntToProcess * 3 * 4; // REVISIT - notice we don't handle duplication, see the note below
                                                                                // REVISIT - *4 for padding, give w = 1
        ( *nUVBufferSize      )[ aArrayIndex ] = aSurfaceCntToProcess * 3 * 2;

        for ( size_t i = 0; i < aSurfaceCntToProcess; ++i ) {
            for ( size_t j = 0; j < 3; ++j ) {

                // element array (vertex index)
                int aVertexIndexNew = i * 3 + j;
                ( *nElementBufferData )[ aArrayIndex ][ aVertexIndexNew ] = ( unsigned short )( aVertexIndexNew );

                // REVISIT - IMPROVE - notice we don't deal with duplication of vertices, which is a big waste of memory space
                //           this can cause problem if the graphics card does not have big memory space
                //           reconsider and rewrite this part
                // vertex array
                int aVertexIndexOld = nElementBufferTmp[ ( aSurfaceBase + i ) * 3 + j ];
                float x = nVertexBufferTmp[ aVertexIndexOld * 3     ];
                float y = nVertexBufferTmp[ aVertexIndexOld * 3 + 1 ];
                float z = nVertexBufferTmp[ aVertexIndexOld * 3 + 2 ];
                ( *nVertexBufferData )[ aArrayIndex ][ aVertexIndexNew * 4     ] = x;
                ( *nVertexBufferData )[ aArrayIndex ][ aVertexIndexNew * 4 + 1 ] = y;
                ( *nVertexBufferData )[ aArrayIndex ][ aVertexIndexNew * 4 + 2 ] = z;
                ( *nVertexBufferData )[ aArrayIndex ][ aVertexIndexNew * 4 + 3 ] = 1.0;

                // uv array
                float u = nUVBufferTmp[ aVertexIndexOld * 2     ];
                float v = nUVBufferTmp[ aVertexIndexOld * 2 + 1 ];
                ( *nUVBufferData )[ aArrayIndex ][ aVertexIndexNew * 2     ] = u;
                ( *nUVBufferData )[ aArrayIndex ][ aVertexIndexNew * 2 + 1 ] = v;

            } // j
        } // i

        aSurfaceBase += aSurfaceCntToProcess;
        aArrayIndex++;
    } // while

    return true;
}

// Convert from QImage to cv::Mat
cv::Mat QImage2Mat( const QImage &nSrc )
{
	cv::Mat tmp( nSrc.height(),
				 nSrc.width(),
				 CV_8UC3,
				 ( uchar* )nSrc.bits(),
				 nSrc.bytesPerLine() );
	cv::Mat result; // deep copy
	cvtColor( tmp, result, CV_BGR2RGB );
	return result;
}

// Convert from cv::Mat to QImage
QImage Mat2QImage( const cv::Mat &nSrc )
{
	cv::Mat tmp;
	cvtColor( nSrc, tmp, CV_BGR2RGB ); // copy and convert color space
	QImage result( ( const uchar* )tmp.data,
				   tmp.cols,
				   tmp.rows,
				   tmp.step,
				   QImage::Format_RGB888 );
	result.bits();	// enforce depp copy, see documentation of
					// QImage::QImage( const uchar *dta, int width, int height, Format format )
	return result;
}

} // namespace ChaoVis
