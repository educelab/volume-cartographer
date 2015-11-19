//
// Created by Ryan Taber on 11/19/15.
//

#ifndef VC_PARSINGHELPERS_H
#define VC_PARSINGHELPERS_H

#include "vc_defines.h"

/*
 * Purpose of File:
 *     - provides helpful parsing methods that are used by multiple
 *       testing files when reading in mesh or point cloud data
 */

class parsingHelpers {

    public:
        void parsePlyFile(std::string filename, std::vector<VC_Vertex>&, std::vector<VC_Cell>&);
        std::vector<std::string> split_string(std::string input);
};


#endif //VC_PARSINGHELPERS_H
