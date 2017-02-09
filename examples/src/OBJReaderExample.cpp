//
// Created by Seth Parker on 2/9/17.
//

#include "core/io/OBJReader.hpp"
#include "core/io/OBJWriter.h"

namespace vc = volcart;

int main(int, char* argv[])
{
    vc::io::OBJReader reader;
    reader.setPath(argv[1]);
    auto mesh = reader.read();
    auto uv = reader.getUVMap();
    auto img = reader.getTextureMat();
}