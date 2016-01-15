#include <iostream>
#include "volumepkg.h"

int main(int argc, char** argv)
{
    if (argc < 5) {
        std::cerr << "Usage:\n";
        std::cerr << "    " << argv[0] << " [volpkg] [x] [y] [z]\n";
        std::exit(1);
    }

    auto path = std::string(argv[1]);
    std::cout << path << std::endl;
    int32_t x = std::stoi(std::string(argv[2]));
    int32_t y = std::stoi(std::string(argv[3]));
    int32_t z = std::stoi(std::string(argv[4]));

    VolumePkg v(path);

    std::cout << v.volume().getStructureTensor(x, y, z) << "\n";
}
