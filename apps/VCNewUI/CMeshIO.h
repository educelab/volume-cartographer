// CMeshIO.h
// Chao Du Sept. 2014
#ifndef _CMESHIO_H_
#define _CMESHIO_H_

#include <string>

#include "CMesh.h"

namespace ChaoVis {

// utility class for mesh input/output
class CMeshIO {

public:
	static bool ReadPlyFile( const std::string &nFileName,
							CMesh &nMesh );
	static bool ReadPlyFileBinary( const std::string &nFileName,
								CMesh &nMesh );
	static bool WritePlyFile( const std::string &nFileName,
							  const CMesh &nMesh );

	static bool ReadObjFile( const std::string &nFileName,
							 CMesh &nMesh );
	static bool WriteObjFile( const std::string &nFileName,
							  const CMesh &nMesh );
	static void parseObjVertexLine( const std::string &nLine,
									double &x,
									double &y,
									double &z );
	static void parseObjFaceLine( const std::string &nLine,
								  int &x,
								  int &y,
								  int &z );
	static void parseObjFaceSubLine( const std::string &nSubLine,
									int &p,
									int &ptex );

}; // class CMeshIO

} // namespace ChaoVis

#endif // _CMESHIO_H_
