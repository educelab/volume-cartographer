// Volume Package Functionality Examples
#include <iostream>

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

    // print the object input
    std::cout << "Volume package name before change: " << volpkg.name()
              << std::endl;

    // change a value
    volpkg.setMetadata("name", "Transformed Name");

    // print the changed object
    std::cout << "Volume package name after change: " << volpkg.name()
              << std::endl;

    // save the new json file to test.json
    volpkg.saveMetadata("test.json");
}
