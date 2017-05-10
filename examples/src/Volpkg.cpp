// Volume Package Functionality Examples
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vc/core/types/VolumePkg.hpp"

std::string volpkgpath = "";

int main(int argc, char* argv[])
{

    if (argc < 2) {
        std::cerr << "Usage:" << std::endl;
        std::cerr << argv[0] << " [volpkg path]" << std::endl;
        exit(EXIT_FAILURE);
    }

    volpkgpath = argv[1];

    if (volpkgpath == "") {
        std::cerr << "ERROR: Incorrect/missing volpkg location!" << std::endl;
        exit(EXIT_FAILURE);
    }

    volcart::VolumePkg volpkg(volpkgpath);
    for (auto v : volpkg.volumes()) {
        std::cout << v->name() << std::endl;
    }

    std::string path;
    for (size_t i = 0; i < volpkg.numberOfVolumes(); i++) {
        path = "test" + std::to_string(i) + ".png";
        cv::imwrite(path, volpkg.volume(i)->getSliceData(0));
    }
}
