#include "vc/core/io/OBJReader.hpp"

namespace vc = volcart;

int main(int, char* argv[])
{
    vc::io::OBJReader reader;
    reader.setPath(argv[1]);
    auto mesh = reader.read();
    auto uv = reader.getUVMap();
    auto img = reader.getTextureMat();
}
