// Volume Package Functionality Examples
#include <iostream>

#include "volumepkg/volumepkg.h"

std::string volpkgpath = "";

int main(int argc, char* argv[]) {
    
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

    VolumePkg volpkg(volpkgpath);

    // print the object input
    std::cout << "Volume package name before change: " << volpkg.getPkgName() << std::endl;

    // change a value
    volpkg.setMetadata("volumepkg name", "Transformed Name");

    // print the changed object
    std::cout << "Volume package name after change: " << volpkg.getPkgName() << std::endl;

    // save the new json file to test.json
    volpkg.saveMetadata("test.json");

}
