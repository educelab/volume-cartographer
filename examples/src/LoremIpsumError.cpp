#include <iostream>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/VolumePkg.hpp"

namespace fs = volcart::filesystem;

int main(int /*argc*/, char** argv)
{
    auto volpkgPath = fs::path(argv[1]);
    auto segID = std::string(argv[2]);
    volcart::VolumePkg volpkg(volpkgPath);
    auto cloud = volpkg.segmentation(segID)->getPointSet();

    double errsum = 0;
    for (auto p : cloud) {
        errsum += std::abs(p[1] - 13);
    }

    std::cout << "num points: " << cloud.size() << std::endl;
    std::cout << "error metric: " << errsum / cloud.size() << std::endl;
}
