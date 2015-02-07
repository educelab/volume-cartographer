// UObjHelper.cpp
// Chao Du 2014 Dec
#include "UObjHelper.h"

#include "objTester/objLoader.h"

namespace ChaoVis {

bool ReadObjFile( const char *nFileName,
                  int *nNumVertex,
                  int *nNumFace,
                  float **nV, /* v1x v1y v1z v2x v2y v2z ... */
                  int **nF,/* f1a f1b f1c f2a f2b f2c ... */
                  float **nUV,
                  std::vector< Vec3< int > > &nFaces,
                  std::vector< PointXYZRGBNormal > &nPoints,
                  std::vector< std::vector< int > > &nVDupList )
{
    // use objLoader
    // http://www.kixor.net/dev/objloader/
    objLoader *objData = new objLoader();

    objData->load( nFileName );

    int aNumVertex = objData->vertexCount;

    // allocate space for the duplicated vertex indices
    nVDupList.resize( aNumVertex );
    PointXYZRGBNormal aPt;
    for ( int i = 0; i < aNumVertex; ++i ) {
        aPt.x = objData->vertexList[ i ]->e[ 0 ];
        aPt.y = objData->vertexList[ i ]->e[ 1 ];
        aPt.z = objData->vertexList[ i ]->e[ 2 ];
        nPoints.push_back( aPt );
    }

    int aNumFace = objData->faceCount;

    // REVISIT - note that the texture coordinates read from obj file does not have the same index with vertex index!
    //           (sometimes) they even don't have the save total number. The correspondence is set in "f v/vt/vn"
    //           And since OpenGL cannot handle this disjoint indexing for vertex and texture, extra step has to be taken
    //           http://www.opengl-tutorial.org/beginners-tutorials/tutorial-7-model-loading/
    //           see the operations above to handle this obj->OpenGL transformation
    //           Since in the "f v/vt/vn", v/vt correspondence may not be consistent, we duplicate the vertex. So
    //           actually we have three times vertex and vertex texture relative to face number, objData->vertexCount
    //           is actually not used.

    *nNumVertex = aNumFace * 3;
    *nNumFace = aNumFace;

    // allocate memory space
//	*nV = new float[ 3/*4*/ * ( *nNumVertex ) ];	// REVISIT - 4 for padding, but really needed???
//	*nF = new int[ 3 * ( *nNumFace ) ];//new int[ 3 * ( *nNumFace ) ];
//	*nUV = new float[ 2 * ( *nNumVertex ) ];
    *nV = new float[ 3/*4*/ * aNumFace * 3 ];	// REVISIT - 4 for padding, but really needed???
    *nF = new int[ 3 * aNumFace ];//new int[ 3 * ( *nNumFace ) ];
    *nUV = new float[ 2 * aNumFace * 3 ];

    int aVertexCounter = 0;
    for ( int i = 0; i < aNumFace; ++i ) {

        for ( int j = 0; j < 3; ++j ) {

            // REVISIT - minus one ( - 1 ) because obj file format index starts from one ( 1 ), not zero ( 0 )
            //           objLoad has already handled this for us.
            ( *nV )[ ( 3 * i + j ) * 3     ] = objData->vertexList[ objData->faceList[ i ]->vertex_index[ j ] ]->e[ 0 ];
            ( *nV )[ ( 3 * i + j ) * 3 + 1 ] = objData->vertexList[ objData->faceList[ i ]->vertex_index[ j ] ]->e[ 1 ];
            ( *nV )[ ( 3 * i + j ) * 3 + 2 ] = objData->vertexList[ objData->faceList[ i ]->vertex_index[ j ] ]->e[ 2 ];

            ( *nUV )[ ( 3 * i + j ) * 2     ] = objData->textureList[ objData->faceList[ i ]->texture_index[ j ] ]->e[ 0 ];
            ( *nUV )[ ( 3 * i + j ) * 2 + 1 ] = objData->textureList[ objData->faceList[ i ]->texture_index[ j ] ]->e[ 1 ];

        }

        // REVISIT - we keep the information of the real vertex indices for each face
        nFaces.push_back( Vec3< int >( objData->faceList[ i ]->vertex_index[ 0 ],
                                        objData->faceList[ i ]->vertex_index[ 1 ],
                                        objData->faceList[ i ]->vertex_index[ 2 ] ) );
        nVDupList[ objData->faceList[ i ]->vertex_index[ 0 ] ].push_back( 3 * i     );
        nVDupList[ objData->faceList[ i ]->vertex_index[ 1 ] ].push_back( 3 * i + 1 );
        nVDupList[ objData->faceList[ i ]->vertex_index[ 2 ] ].push_back( 3 * i + 2 );

        // the face for OpenGL rendering
        ( *nF )[ i * 3     ] = 3 * i;
        ( *nF )[ i * 3 + 1 ] = 3 * i + 1;
        ( *nF )[ i * 3 + 2 ] = 3 * i + 2;

    }

    // clean up
    delete objData;

    return true;
}

bool SaveObjFile( const char *nFileName,
                  int nNumVertex,
				  int nNumFace,
				  float *nV,
				  float *nF,
				  float *nUV )
{
    // REVISIT - FILL ME HERE
	return false;
}

} // namespace ChaoVis
