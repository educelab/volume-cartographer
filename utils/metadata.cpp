// VC Metadata Viewer/Editor
#include <iostream>

#include "volumepkg.h"

std::string volpkgpath = "";

int main(int argc, char* argv[]) {
    std::cout << "vc_metadata" << std::endl;
    if (argc < 3) {
        std::cerr << "Usage:" << std::endl;
        std::cerr << argv[0] << " [volpkg] key=value" << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << std::endl;

    volpkgpath = argv[1];

    if (volpkgpath == "") {
        std::cerr << "ERROR: Incorrect/missing volpkg location!" << std::endl;
        exit(EXIT_FAILURE);
    }

    VolumePkg volpkg(volpkgpath);

    std::map<std::string, std::string> parsedMetadata;

    // Parse the metadata key and its value
    for (int i = 2; i < argc; ++i) {
        std::string argument = argv[i];
        std::size_t delimiter = argument.find("=");
        std::string key = argument.substr(0, delimiter);
        std::string value = argument.substr(delimiter+1, argument.npos);

        parsedMetadata[key] = value;
    }

    // Attempt to set metadata keys to specified values
    std::map<std::string, std::string>::iterator parsedIterator;
    for (parsedIterator = parsedMetadata.begin(); parsedIterator != parsedMetadata.end(); ++parsedIterator){
        std::cout << "Attempting to set key \"" << (*parsedIterator).first << "\" to value \"" << (*parsedIterator).second << "\"" <<  std::endl;
        if(volpkg.setMetadata((*parsedIterator).first, (*parsedIterator).second) == EXIT_SUCCESS) {
            std::cout << "Key set successfully." << std::endl;
        }
        std::cout << std::endl;
    }

    // save the new json file to test.json
    //volpkg.saveMetadata("test.json");
    return EXIT_SUCCESS;
}