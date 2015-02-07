// CMeshGL.h
// Chao Du 2014 Dec
#ifndef _CMESHGL_H_
#define _CMESHGL_H_

#include "CMesh.h"
#include "C3DObj.h"

#ifdef _WINDOWS
#include <Windows.h>
#endif // _WINDOWS
#include <GL/gl.h>

namespace ChaoVis {

class CXCurve;

// REVISIT - NOTE - Chao 20141225, this is where multiple inheritance happen
//           may not be the best design, especially for implementation in
//           other language, we may want to change C3DObj an interface
class CMeshGL : public C3DObj< float >, public CMesh {

public:
    CMeshGL( void );
    ~CMeshGL( void );

    virtual void Draw( void );

    bool ReadModel( const std::string &nModelFileName );
	bool SaveModel( const std::string &nModelFileName );

	void ComposeVirtualRectangle( int nSliceIndex,	// x
								 int nImgW,			// y
								 int nImgH );		// z

	void ChangeVertex( int nIndex, 
					   const Vec3< float > &nPos );
	void ChangeVertex( const ChaoVis::CXCurve *nCurve,
						int nSliceIndex );

    void SetTextureImage( const unsigned char *nSrcImg,
                          int nWidth,
                          int nHeight,
                          int nChannel );
    void SetTexture( GLuint nTextureId );

    Vec3< float > GetLB( void ) { return fLB; }
    Vec3< float > GetUB( void ) { return fUB; }

protected:

private:
    GLuint fTextureId;
    int fElementArrayNum;

    int fNumVertices;
    int fNumFaces;

    GLuint *fVertexBufferId;
    GLuint *fUVBufferId;
    GLuint *fElementBufferId;

    GLfloat **fVertexBufferData;
    GLushort **fElementBufferData;
    GLfloat **fUVBufferData;

    int *fVertexBufferSize;
    int *fUVBufferSize;
    int *fElementBufferSize;

    Vec3< float > fLB, fUB;

    std::vector< std::vector< int > > fVDupList; // REVISIT - for Obj file format, see TODO.txt for explanation

    unsigned char *fTextureImg;

}; // class CMeshGL

} // namespace ChaoVis

#endif // _CMESHGL_H_
