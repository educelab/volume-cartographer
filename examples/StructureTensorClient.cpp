#include <iostream>
#include <algorithm>
#include "volumepkg.h"

int main(int argc, char** argv)
{
    if (argc < 5) {
        std::cerr << "Usage:\n";
        std::cerr << "    " << argv[0] << " volpkg x y z [radius]\n";
        std::exit(1);
    }

    int32_t radius = 1;
    if (argc > 5) {
        radius = std::stoi(argv[5]);
    }

    VolumePkg vpkg{std::string(argv[1])};
    volcart::Volume v = vpkg.volume();
    int32_t x = std::stoi(argv[2]);
    int32_t y = std::stoi(argv[3]);
    int32_t z = std::stoi(argv[4]);

    std::cout << "structure tensor:\n"
              << v.structureTensorAtIndex(x, y, z, radius) << "\n";
    try {
        auto pairs = v.eigenPairsAtIndex(x, y, z, radius);
        std::cout << "eigenvalues/eigenvectors\n";
        std::for_each(pairs.begin(), pairs.end(),
                      [](const std::pair<EigenValue, EigenVector>& p) {
                          std::cout << p.first << ":  " << p.second << "\n";
                      });
    } catch (const volcart::ZeroStructureTensorException& ex) {
        std::cout << ex.what() << std::endl;
    }
}
