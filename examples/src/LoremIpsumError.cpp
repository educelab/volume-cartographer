#include "volumepkg/volumepkg.h"
#include <boost/filesystem.hpp>
#include <iostream>

namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
    auto volpkgPath = fs::path(argv[1]);
    auto segID = std::string(argv[2]);
    VolumePkg volpkg(volpkgPath);
    volpkg.setActiveSegmentation(segID);
    auto cloud = volpkg.openPCDCloud();

    double errsum = 0;
    for (auto p : *cloud) {
        errsum += std::abs(p.y - 13);
    }

    std::cout << "num points: " << cloud->size() << std::endl;
    std::cout << "error metric: " << errsum / cloud->size() << std::endl;
}
