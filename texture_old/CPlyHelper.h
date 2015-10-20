// CPlyHelper.h
// Chao Du Sept. 2014
#ifndef _CPLYHELPER_H_
#define _CPLYHELPER_H_

#include <string>

#include "CMesh.h"

namespace ChaoVis {

class CPlyHelper {

public:
	static bool ReadPlyFile( const std::string &nFileName,
							CMesh &nMesh );
	static bool WritePlyFile( const std::string &nFileName,
							const CMesh &nMesh ); 

private:

}; // class CPlyHelper

} // namespace ChaoVis

#endif // _CPLYHELPER_H_
