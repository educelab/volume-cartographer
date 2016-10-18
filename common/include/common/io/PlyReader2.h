//
// Created by Hannah Hatch on 10/18/16.
//

#pragma once

#include <fstream>
#include <iostream>

#include "boost/filesystem/path.hpp"
#include "common/vc_defines.h"

bool PLYReader(boost::filesystem::path path, volcart::ITKMesh::Pointer mesh);