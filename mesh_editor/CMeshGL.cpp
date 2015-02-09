// CMeshGL.cpp
// Chao Du 2014 Dec
#include "HBase.h"
#include "CMeshGL.h"

#include "UDataManipulateUtils.h"
#include "UObjHelper.h"
#include "CPlyHelper.h"

#include "CXCurve.h"

//#include <gl/glew.h>  // REVISIT - GL_TEXTURE0

using namespace ChaoVis;

// Constructor
CMeshGL::CMeshGL( void ) :
    fVertexBufferId     ( NULL ),
    fUVBufferId         ( NULL ),
    fElementBufferId    ( NULL ),
    fVertexBufferData   ( NULL ),
    fUVBufferData       ( NULL ),
    fElementBufferData  ( NULL ),
    fVertexBufferSize   ( NULL ),
    fUVBufferSize       ( NULL ),
    fElementBufferSize  ( NULL ),
    fElementArrayNum( 0 ),
    fTextureImg         ( NULL )
{
	// REVISIT - we'll allocate memory space when we actually read data (or set fake data);
	//           we don't need to pre-allocate memory space
//    fElementArrayNum = 1;
//                            //           calculate the space we need
//    fVertexBufferId     = new GLuint[ fElementArrayNum ];
//    fUVBufferId         = new GLuint[ fElementArrayNum ];
//    fElementBufferId    = new GLuint[ fElementArrayNum ];

//    fVertexBufferData   = new GLfloat*[ fElementArrayNum ];
//    fUVBufferData       = new GLfloat*[ fElementArrayNum ];
//    fElementBufferData  = new GLushort*[ fElementArrayNum ];

//    fVertexBufferSize   = new int[ fElementArrayNum ];
//    fUVBufferSize       = new int[ fElementArrayNum ];
//    fElementBufferSize  = new int[ fElementArrayNum ];
}

// Destructor
CMeshGL::~CMeshGL( void )
{
    deleteNULL( fVertexBufferId, true );
    deleteNULL( fUVBufferId, true );
    deleteNULL( fElementBufferId, true );

    for ( int i = 0; i < fElementArrayNum; ++i ) {
        deleteNULL( fVertexBufferData[ i ], true );
        deleteNULL( fUVBufferData[ i ], true );
        deleteNULL( fElementBufferData[ i ], true );
    }
    deleteNULL( fVertexBufferData, true );
    deleteNULL( fUVBufferData, true );
    deleteNULL( fElementBufferData, true );

    deleteNULL( fVertexBufferSize, true );
    deleteNULL( fUVBufferSize, true );
    deleteNULL( fElementBufferSize, true );

    deleteNULL( fTextureImg, true );
}

// Draw OpenGL content
void CMeshGL::Draw( void )
{
    // texture
//    glActiveTexture( GL_TEXTURE0 ); // REVISIT - need glew.h, >OpenGL2.0

    // rotation and translation
    glPushMatrix();

    glTranslatef( fT[ 0 ], fT[ 1 ], fT[ 2 ] );

    // REVISIT - pass data pointer
/*
    GLfloat m[ 16 ] = { 1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0, 1.0 };
*/
    GLfloat m[ 16 ] = { fR[ 0 ][ 0 ], fR[ 0 ][ 1 ], fR[ 0 ][ 2 ], 0.0,
                        fR[ 1 ][ 0 ], fR[ 1 ][ 1 ], fR[ 1 ][ 2 ], 0.0,
                        fR[ 2 ][ 0 ], fR[ 2 ][ 1 ], fR[ 2 ][ 2 ], 0.0,
                        0.0, 0.0, 0.0, 1.0 };
    glMultMatrixf( m /* fR.constData() */ );

//    GLfloat faceColor[ 4 ] = { 0.4, 0.0, 1.0, 0.0 };
//    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, faceColor );
//	glColor4f( 1.0, 1.0, 1.0, 1.0 );

    // draw data
    for ( int i = 0; i < fElementArrayNum; ++i ) {

        // REVISIT - all of the operations below need glew.h
        // setting up the vertex array
//        glBindBuffer( GL_ARRAY_BUFFER, fVertexBufferId[ i ] );

        // setting up texture coordinates
//        glBindBuffer( GL_ARRAY_BUFFER, fUVBufferId[ i ] );

        // submitting the renderign job
//        glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, fElementBufferId[ i ] );

		// REVISIT - DEBUG
//        assert( fVertexBufferData[ i ] != NULL && fUVBufferData[ i ] != NULL );

        glVertexPointer( 4, GL_FLOAT, /*4 * sizeof( GL_FLOAT )*/0, fVertexBufferData[ i ] );
        glTexCoordPointer( 2, GL_FLOAT, /*2 * sizeof( GL_FLOAT )*/0, fUVBufferData[ i ] );
        glEnableClientState( GL_VERTEX_ARRAY );
        glEnableClientState( GL_TEXTURE_COORD_ARRAY );
//		glEnableClientState( GL_NORMAL_ARRAY );

		// REVISIT - DEBUG
//        assert( fElementBufferSize[ i ] != NULL && fElementBufferData[ i ] != NULL );

        glBindTexture( GL_TEXTURE_2D, fTextureId );
        // core content of drawing code
        glDrawElements( GL_TRIANGLES,               // mode
                        fElementBufferSize[ i ],    // count
                        GL_UNSIGNED_SHORT,          // type
                        ( void* )fElementBufferData[ i ] );               // element array buffer offset

    }

//    glDisableClientState( GL_VERTEX_ARRAY );
//    glDisableClientState( GL_TEXTURE_COORD_ARRAY );
    //		glDisableClientState( GL_NORMAL_ARRAY );
    glPopMatrix();
}

// Read model
bool CMeshGL::ReadModel( const std::string &nModelFileName )
{
    int aNumVertex, aNumFace;

    int *aElementBufferTmp = NULL;
    float *aVertexBufferTmp = NULL;
    float *aUVBufferTmp = NULL;

    if ( nModelFileName.find( ".ply" ) != std::string::npos ) {
        // read ply file
        CPlyHelper::ReadPlyFile( nModelFileName, *this );

        aNumVertex = fFaces.size() * 3;//fPoints.size();
        aNumFace = fFaces.size();

        fVDupList.resize( fPoints.size() );

        aVertexBufferTmp = new float[ 3 * aNumFace * 3 ];
        aElementBufferTmp = new int[ 3 * aNumFace ];
        aUVBufferTmp = new float[ 2 * aNumFace * 3 ];

        fLB = Vec3< float >( 1e6, 1e6, 1e6 );
        fUB = Vec3< float >( -1e6, -1e6, -1e6 );

        for ( int i = 0; i < aNumFace; ++i ) {

            for ( int j = 0; j < 3; ++j ) {
                aVertexBufferTmp[ ( 3 * i + j ) * 3     ] = fPoints[ fFaces[ i ][ j ] ].x;
                aVertexBufferTmp[ ( 3 * i + j ) * 3 + 1 ] = fPoints[ fFaces[ i ][ j ] ].y;
                aVertexBufferTmp[ ( 3 * i + j ) * 3 + 2 ] = fPoints[ fFaces[ i ][ j ] ].z;

                aUVBufferTmp[ ( 3 * i + j ) * 2     ] = 0.0; // REVISIT - no texture attatcheed
                aUVBufferTmp[ ( 3 * i + j ) * 2 + 1 ] = 0.0;

                // find bounding box
                if ( fPoints[ fFaces[ i ][ j ] ].x < fLB[ 0 ] ) {
                    fLB[ 0 ] = fPoints[ fFaces[ i ][ j ] ].x;
                }
                if ( fPoints[ fFaces[ i ][ j ] ].x > fUB[ 0 ] ) {
                    fUB[ 0 ] = fPoints[ fFaces[ i ][ j ] ].x;
                }
                if ( fPoints[ fFaces[ i ][ j ] ].y < fLB[ 1 ] ) {
                    fLB[ 1 ] = fPoints[ fFaces[ i ][ j ] ].y;
                }
                if ( fPoints[ fFaces[ i ][ j ] ].y > fUB[ 1 ] ) {
                    fUB[ 1 ] = fPoints[ fFaces[ i ][ j ] ].y;
                }
                if ( fPoints[ fFaces[ i ][ j ] ].z < fLB[ 2 ] ) {
                    fLB[ 2 ] = fPoints[ fFaces[ i ][ j ] ].z;
                }
                if ( fPoints[ fFaces[ i ][ j ] ].z > fUB[ 2 ] ) {
                    fUB[ 2 ] = fPoints[ fFaces[ i ][ j ] ].z;
                }
            }

            fVDupList[ fFaces[ i ][ 0 ] ].push_back( 3 * i     );
            fVDupList[ fFaces[ i ][ 1 ] ].push_back( 3 * i + 1 );
            fVDupList[ fFaces[ i ][ 2 ] ].push_back( 3 * i + 2 );

            aElementBufferTmp[ i * 3     ] = i * 3;     //fFaces[ i ][ 0 ];
            aElementBufferTmp[ i * 3 + 1 ] = i * 3 + 1; //fFaces[ i ][ 1 ];
            aElementBufferTmp[ i * 3 + 2 ] = i * 3 + 2; //fFaces[ i ][ 2 ];
        }

    } else if ( nModelFileName.find( ".obj" ) != std::string::npos ) {
        // read obj file
        if ( !ReadObjFile( nModelFileName.c_str(),
                            &aNumVertex,
                            &aNumFace,
                            &aVertexBufferTmp,
                            &aElementBufferTmp,
                            &aUVBufferTmp,
                           fFaces,
                           fPoints,
                           fVDupList ) ) {
            // REVISIT - maybe need to clean up the memory
            return false;
        }

        // find bounding box
        for ( size_t j = 0; j < fPoints.size(); ++j ) {

            // find bounding box
            if ( fPoints[ j ].x < fLB[ 0 ] ) {
                fLB[ 0 ] = fPoints[ j ].x;
            }
            if ( fPoints[ j ].x > fUB[ 0 ] ) {
                fUB[ 0 ] = fPoints[ j ].x;
            }
            if ( fPoints[ j ].y < fLB[ 1 ] ) {
                fLB[ 1 ] = fPoints[ j ].y;
            }
            if ( fPoints[ j ].y > fUB[ 1 ] ) {
                fUB[ 1 ] = fPoints[ j ].y;
            }
            if ( fPoints[ j ].z < fLB[ 2 ] ) {
                fLB[ 2 ] = fPoints[ j ].z;
            }
            if ( fPoints[ j ].z > fUB[ 2 ] ) {
                fUB[ 2 ] = fPoints[ j ].z;
            }
        }

        // besides read the data for OpenGL, we need to store the data in CMesh
        // REVISIT - UPDATE 2015-02-04, this is also done in ReadObjFile()
        //           So now { fFaces, fPoints, fEdges } are original information
        //           before duplication; { aElementBufferTmp, aVertexBufferTmp, aUVBufferTmp }
        //           are suitable for OpenGL rendering with vertex duplication.
        int p1, p2, p3;
        for ( size_t i = 0; i < fFaces.size(); ++i ) {
            // REVISIT - why do we sort the points at all?
            p1 = fFaces[ i ][ 0 ]; p2 = fFaces[ i ][ 1 ]; p3 = fFaces[ i ][ 2 ];
            fEdges.insert( p1 > p2 ? Vec2< int >( p2, p1 ) : Vec2< int >( p1, p2 ) );
            fEdges.insert( p2 > p3 ? Vec2< int >( p3, p2 ) : Vec2< int >( p2, p3 ) );
            fEdges.insert( p3 > p1 ? Vec2< int >( p1, p3 ) : Vec2< int >( p3, p1 ) );
        }
    }


    // split the data to make it suitable for rendering in OpenGL
    if ( !SplitVertexAndElementBuffer( aNumVertex,
                                       aNumFace,
                                       aElementBufferTmp,
                                       &fElementBufferData,//&nData.g_element_buffer_data,
                                       aVertexBufferTmp,
                                       &fVertexBufferData,//&nData.g_vertex_buffer_data,
                                       aUVBufferTmp,
                                       &fUVBufferData,//&nData.g_uv_buffer_data,
                                       &fElementBufferSize,//&nData.g_element_buffer_size,
                                       &fVertexBufferSize,//&nData.g_vertex_buffer_size,
                                       &fUVBufferSize,//&nData.g_uv_buffer_size,
                                       &fElementArrayNum ) ) {//&nData.g_element_array_num ) ) {
        // clean up
        delete[ ]aElementBufferTmp;
        delete[ ]aVertexBufferTmp;
        delete[ ]aUVBufferTmp;
        return false;
    }

    delete[ ]aElementBufferTmp;
    delete[ ]aVertexBufferTmp;
    delete[ ]aUVBufferTmp;

    fNumVertices = aNumVertex;
    fNumFaces = aNumFace;

    return true;
}

// Save model
bool CMeshGL::SaveModel( const std::string &nModelFileName )
{
	// REVISIT - FILL ME HERE
//	bool aIsOK = SaveObjFile( nModelFileName.c_str(),
//	                          0,
//				              0,
//				              NULL,
//				              NULL,
//				              NULL );
    bool aIsOk = CPlyHelper::WritePlyFile( nModelFileName, *this );
    return aIsOk;
}

// Compose a virtual rectangle
void CMeshGL::ComposeVirtualRectangle( int nSliceIndex,	// x
										int nImgW,			// y
										int nImgH )		// z
{
	// (1) fill the member variables
	PointXYZRGBNormal aPt;
    aPt.x = nSliceIndex; aPt.y = 0.0;   aPt.z = 0.0;   fPoints.push_back( aPt );
    aPt.x = nSliceIndex; aPt.y = 0.0;   aPt.z = nImgH; fPoints.push_back( aPt );
    aPt.x = nSliceIndex; aPt.y = nImgW; aPt.z = 0.0;   fPoints.push_back( aPt );
    aPt.x = nSliceIndex; aPt.y = nImgW; aPt.z = nImgH; fPoints.push_back( aPt );

    fFaces.push_back( Vec3< int >( 0, 1, 2 ) );
	fFaces.push_back( Vec3< int >( 1, 3, 2 ) );

	// (2) fill in the memory space
	size_t aSurfaceCntToProcess = 2; // use 2 triangles to represent the rectangle
	size_t aArraySize = 1; // only need 1 element to store the vertices, etc.

	fElementArrayNum = aArraySize;

	fElementBufferData = new unsigned short*[ aArraySize ];
	fVertexBufferData = new float*[ aArraySize ];
	fUVBufferData = new float*[ aArraySize ];

	fElementBufferSize = new int[ aArraySize ];
	fVertexBufferSize = new int[ aArraySize ];
	fUVBufferSize = new int[ aArraySize ];

	fElementBufferData[ 0 ] = new unsigned short[ aSurfaceCntToProcess * 3 ];
	fVertexBufferData[ 0 ] = new float[ aSurfaceCntToProcess * 3 * 4 ];
	fUVBufferData[ 0 ] = new float[ aSurfaceCntToProcess * 3 * 2 ];

	fElementBufferSize[ 0 ] = aSurfaceCntToProcess * 3;
	fVertexBufferSize[ 0 ] = aSurfaceCntToProcess * 3 * 4;
	fUVBufferSize[ 0 ] = aSurfaceCntToProcess * 3 * 2;

	fElementBufferData[ 0 ] = new unsigned short[ aSurfaceCntToProcess * 3 ];

    int aCoordCombo[ 4 ][ 2 ] = { { 0, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 } };
    // REVISIT - for the slice plane, we have to flip the texture coordinates because when the slice image
    //           is converted to create the texture, it's not flipped, where OpenGL will treat the left bottom corner
    //           of the original normal image as the top left corner. So we need to either flip the image, or flip the
    //           texture coordinates to make the image shown correct.
    int aCoordComboFlipped[ 4 ][ 2 ] = { { 0, 1 }, { 0, 0 }, { 1, 1 }, { 1, 0 } };
	int aVertexIndexCombo[ 2 ][ 3 ] = { { 0, 1, 2 }, { 1, 3, 2 } };


	for ( size_t i = 0; i < aSurfaceCntToProcess; ++i ) {

		for ( size_t j = 0; j < 3; ++j ) {

//			int aVertexIndexNew = aVertexIndexCombo[ i ][ j ];
            int aVertexIndexNew = i * 3 + j; // REVISIT - this is where double vertices and double edges come from
                                                //        the vertices are duplicated and filled in memory sequentially

			fElementBufferData[ 0 ][ aVertexIndexNew ] = ( unsigned short )( aVertexIndexNew );

			fVertexBufferData[ 0 ][ aVertexIndexNew * 4 ] = nSliceIndex;
			fVertexBufferData[ 0 ][ aVertexIndexNew * 4 + 1 ] = aCoordCombo[ aVertexIndexCombo[ i ][ j ] ][ 0 ] * nImgW;
            fVertexBufferData[ 0 ][ aVertexIndexNew * 4 + 2 ] = aCoordCombo[ aVertexIndexCombo[ i ][ j ] ][ 1 ] * nImgH;
			fVertexBufferData[ 0 ][ aVertexIndexNew * 4 + 3 ] = 1.0;

            // REVISIT - 20150202 the problem of the texture could not be rendered, probably is the endianess problem
            //           e.g. we specified 1.0 as the coordinate, which should be 00111111100000000000000000000000 (binary)
            //           or 0x3f800000 (hex), however, the data stored here and read in Draw() function is
            //           0000 0000 0000 0000 1000 0000 0011 1111 (binary), or 0x0000803f (hex), which is 1e-41.
            //           So the code may work on Windows, but it may not work on Unix/Linux properly.
            // REVISIT - the above statement seems not correct, according to http://ubuntuforums.org/showthread.php?t=1958055
            //           both Windows and Linux based on Intel CPU are little endian, that means the least significant byte
            //           is stored at lower memory address, which seems correct for the above case, 0x0000803f (hex)
            //           is stored in the order of least significant byte to most significant byte.
            fUVBufferData[ 0 ][ aVertexIndexNew * 2 ] = aCoordComboFlipped[ aVertexIndexCombo[ i ][ j ] ][ 0 ];
            fUVBufferData[ 0 ][ aVertexIndexNew * 2 + 1 ] = aCoordComboFlipped[ aVertexIndexCombo[ i ][ j ] ][ 1 ];

		}
    }
}

// Change a vertex data by index
void CMeshGL::ChangeVertex( int nIndex,
							const Vec3< float > &nPos )
{
	// update points
	fPoints[ nIndex ].x = nPos[ 0 ];
	fPoints[ nIndex ].y = nPos[ 1 ];
	fPoints[ nIndex ].z = nPos[ 2 ];

	// update data in memory
	const unsigned short MAX_NUM_FACE_IN_ARRAY = USHORT_SIZE / 3;
    const unsigned short MAX_NUM_VERTEX_IN_ARRAY = MAX_NUM_FACE_IN_ARRAY * 3;
	// an alternative way to determine the point's array index is using fVertexBufferSize
    int aArrayIndex;// = nIndex / ( MAX_NUM_FACE_IN_ARRAY * 3 );
    int aVertexIndex;// = nIndex % ( MAX_NUM_FACE_IN_ARRAY * 3 );

    // REVISIT - FILL ME HERE - use the fVDupList to determine the index
    for ( size_t i = 0; i < fVDupList[ nIndex ].size(); ++i ) {

        aArrayIndex = fVDupList[ nIndex ][ i ] / MAX_NUM_VERTEX_IN_ARRAY;
        aVertexIndex = fVDupList[ nIndex ][ i ] % MAX_NUM_VERTEX_IN_ARRAY;

        fVertexBufferData[ aArrayIndex ][ aVertexIndex * 4     ] = nPos[ 0 ];
        fVertexBufferData[ aArrayIndex ][ aVertexIndex * 4 + 1 ] = nPos[ 1 ];
        fVertexBufferData[ aArrayIndex ][ aVertexIndex * 4 + 2 ] = nPos[ 2 ];
//        fVertexBufferData[ aArrayIndex ][ aVertexIndex * 4     ] = nPos[ 0 ];
//        fVertexBufferData[ aArrayIndex ][ aVertexIndex * 4 + 1 ] = nPos[ 1 ];
//        fVertexBufferData[ aArrayIndex ][ aVertexIndex * 4 + 2 ] = nPos[ 2 ];
        // the 4th component is 1.0;
        // next, tell 3D view to update (outside this function)
        // REVISIT - probably we need to bind buffer again, etc...
    }
}

// Change the vertices corresponding to the whole curve
void CMeshGL::ChangeVertex( const CXCurve *nCurve,
							int nSliceIndex )
{
    for ( size_t i = 0; i < nCurve->GetPointsNum(); ++i ) {
		ChangeVertex( nCurve->Get3DIndex( i ),
						Vec3< float >( nSliceIndex, nCurve->GetPoint( i )[ 0 ], nCurve->GetPoint( i )[ 1 ] ) );
	}
}

// Set texture image data
void CMeshGL::SetTextureImage( const unsigned char *nSrcImg,
                                  int nWidth,
                                  int nHeight,
                                  int nChannel )
{
    // REVISIT - FILL ME HERE
    if ( fTextureImg != NULL ) {
        deleteNULL( fTextureImg, true );
    }
    fTextureImg = new unsigned char[ nWidth * nHeight * nChannel ];
    memcpy( fTextureImg, nSrcImg, sizeof( unsigned char ) * nWidth * nHeight * nChannel );

    // create OpenGL texture
    glGenTextures( 1, &fTextureId );
    glBindTexture( GL_TEXTURE_2D, fTextureId );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,     GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,     GL_CLAMP_TO_EDGE );
    glTexImage2D( GL_TEXTURE_2D, 0,             /* target, level */
                  GL_RGB,                      /* internal format */
                  nWidth, nHeight, 0,           /* width, height, border */
                  GL_RGB, GL_UNSIGNED_BYTE,    /* external format, type */
                  static_cast< void * >( fTextureImg ) );                /* pixels */
}

// Set texture by id
void CMeshGL::SetTexture( GLuint nTextureId )
{
    fTextureId = nTextureId;
}
