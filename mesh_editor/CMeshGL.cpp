// CMeshGL.cpp
// Chao Du 2014 Dec
#include "HBase.h"
#include "CMeshGL.h"

#include "UDataManipulateUtils.h"
#include "UObjHelper.h"

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
    fElementArrayNum( 0 )
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
    deleteNULL( fVertexBufferId );
    deleteNULL( fUVBufferId );
    deleteNULL( fElementBufferId );

    for ( int i = 0; i < fElementArrayNum; ++i ) {
        deleteNULL( fVertexBufferData[ i ] );
        deleteNULL( fUVBufferData[ i ] );
        deleteNULL( fElementBufferData[ i ] );
    }
    deleteNULL( fVertexBufferData );
    deleteNULL( fUVBufferData );
    deleteNULL( fElementBufferData );

    deleteNULL( fVertexBufferSize );
    deleteNULL( fUVBufferSize );
    deleteNULL( fElementBufferSize );
}

// Draw OpenGL content
void CMeshGL::Draw( void )
{
    // texture
//    glActiveTexture( GL_TEXTURE0 ); // REVISIT - need glew.h, >OpenGL2.0
//    glBindTexture( GL_TEXTURE_2D, fTextureId );

    // rotation and translation
    glPushMatrix();

//	glTranslatef( 0.0, 0.0, -40.0 );
    glTranslatef( -88.0, -99.0, -80.0 ); // position the scene in the center
                                     // REVISIT - FILL ME HERE - hard coded

    // REVISIT - pass data pointer
    GLfloat m[ 16 ] = { 1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0, 1.0 };
    glMultMatrixf( m /* fR.constData() */ );

    GLfloat faceColor[ 4 ] = { 0.4, 0.0, 1.0, 0.0 };
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, faceColor );
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
		assert( fVertexBufferData[ i ] != NULL );

		glVertexPointer( 4, GL_FLOAT, 0, fVertexBufferData[ i ] );

		glEnableClientState( GL_VERTEX_ARRAY );
//		glEnableClientState( GL_NORMAL_ARRAY );

		// REVISIT - DEBUG
		assert( fElementBufferSize[ i ] != NULL && fElementBufferData[ i ] != NULL );

        // core content of drawing code
        glDrawElements( GL_TRIANGLES,               // mode
                        fElementBufferSize[ i ],    // count
                        GL_UNSIGNED_SHORT,          // type
                        ( void* )fElementBufferData[ i ] );               // element array buffer offset

		glDisableClientState( GL_VERTEX_ARRAY );
//		glDisableClientState( GL_NORMAL_ARRAY );
    }

    glPopMatrix();
}

// Read model
bool CMeshGL::ReadModel( const std::string &nModelFileName )
{
    int aNumVertex, aNumFace;

    int *aElementBufferTmp = NULL;
    float *aVertexBufferTmp = NULL;
    float *aUVBufferTmp = NULL;

    if ( !ReadObjFile( nModelFileName.c_str(),
                        &aNumVertex,
                        &aNumFace,
                        &aVertexBufferTmp,
                        &aElementBufferTmp,
                        &aUVBufferTmp ) ) {
        // REVISIT - maybe need to clean up the memory
        return false;
    }

	// besides read the data for OpenGL, we need to store the data in CMesh
	PointXYZRGBNormal aPt;
	int p1, p2, p3;
	for ( int i = 0; i < aNumFace; ++i ) {
		
		for ( int j = 0; j < 3; ++j ) {
			aPt.x = aVertexBufferTmp[ ( 3 * i + j ) * 3     ];
			aPt.y = aVertexBufferTmp[ ( 3 * i + j ) * 3 + 1 ];
			aPt.z = aVertexBufferTmp[ ( 3 * i + j ) * 3 + 2 ];
			fPoints.push_back( aPt );
		}

		p1 = aElementBufferTmp[ i * 3     ];
		p2 = aElementBufferTmp[ i * 3 + 1 ];
		p3 = aElementBufferTmp[ i * 3 + 2 ];
		fFaces.push_back( Vec3< int >( p1, p2, p3 ) );
		fEdges.insert( p1 > p2 ? Vec2< int >( p2, p1 ) : Vec2< int >( p1, p2 ) );
		fEdges.insert( p2 > p3 ? Vec2< int >( p3, p2 ) : Vec2< int >( p2, p3 ) );
		fEdges.insert( p3 > p1 ? Vec2< int >( p1, p3 ) : Vec2< int >( p3, p1 ) );
	}

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

// Compose a virtual rectangle
void CMeshGL::ComposeVirtualRectangle( int nSliceIndex,	// x
										int nImgW,			// y
										int nImgH )		// z
{
	// (1) fill the member variables
	PointXYZRGBNormal aPt;
	aPt.x = nSliceIndex; aPt.y = 0.0; aPt.z = 0.0;
	fPoints.push_back( aPt );
	aPt.x = nSliceIndex; aPt.y = 0.0; aPt.z = nImgH;
	fPoints.push_back( aPt );
	aPt.x = nSliceIndex; aPt.y = nImgW; aPt.z = 0.0;
	fPoints.push_back( aPt );
	aPt.x = nSliceIndex; aPt.y = nImgW; aPt.z = nImgH;
	fPoints.push_back( aPt );

	fFaces.push_back( Vec3< int >( 0, 1, 2) );
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
	int aVertexIndexCombo[ 2 ][ 3 ] = { { 0, 1, 2 }, { 1, 3, 2 } };

	for ( size_t i = 0; i < aSurfaceCntToProcess; ++i ) {

		for ( size_t j = 0; j < 3; ++j ) {

			int aVertexIndexNew = aVertexIndexCombo[ i ][ j ];

			fElementBufferData[ 0 ][ aVertexIndexNew ] = ( unsigned short )( aVertexIndexNew );

			fVertexBufferData[ 0 ][ aVertexIndexNew * 4 ] = nSliceIndex;
			fVertexBufferData[ 0 ][ aVertexIndexNew * 4 + 1 ] = aCoordCombo[ aVertexIndexCombo[ i ][ j ] ][ 0 ] * nImgW;
			fVertexBufferData[ 0 ][ aVertexIndexNew * 4 + 2 ] = aCoordCombo[ aVertexIndexCombo[ i ][ j ] ][ 1 ] * nImgH;
			fVertexBufferData[ 0 ][ aVertexIndexNew * 4 + 3 ] = 1.0;

			fUVBufferData[ 0 ][ aVertexIndexNew * 2 ] = aCoordCombo[ aVertexIndexCombo[ i ][ j ] ][ 0 ];
			fUVBufferData[ 0 ][ aVertexIndexNew * 2 + 1 ] = aCoordCombo[ aVertexIndexCombo[ i ][ j ] ][ 1 ];

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
	// an alternative way to determine the point's array index is using fVertexBufferSize
	int aArrayIndex = nIndex / ( MAX_NUM_FACE_IN_ARRAY * 3 );
	int aVertexIndex = nIndex % ( MAX_NUM_FACE_IN_ARRAY * 3 );
	fVertexBufferData[ aArrayIndex ][ aVertexIndex * 4     ] = nPos[ 0 ];
	fVertexBufferData[ aArrayIndex ][ aVertexIndex * 4 + 1 ] = nPos[ 1 ];
	fVertexBufferData[ aArrayIndex ][ aVertexIndex * 4 + 2 ] = nPos[ 2 ];
	// the 4th component is 1.0; 
	// next, tell 3D view to update (outside this function)
	// REVISIT - probably we need to bind buffer again, etc...
}
