// VC Metadata Viewer/Editor
#include <iostream>

#include "volumepkg.h"

std::string volpkgpath = "";
std::string mode = "";

int main(int argc, char* argv[]) {
    std::cout << "vc_metaedit" << std::endl;
    if (argc < 3) {
        std::cerr << "Usage:" << std::endl;
        std::cerr << argv[0] << " [volpkg] [mode] {key=value}" << std::endl;
        std::cerr << "  MODES:" << std::endl;
        std::cerr << "      -p | print current metadata" << std::endl;
        std::cerr << "      -t | test metadata changes but do not write to file" << std::endl;
        std::cerr << "      -w | write metadata changes to file" << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << std::endl;

    volpkgpath = argv[1];
    mode = argv[2];

    if (volpkgpath == "") {
        std::cerr << "ERROR: Incorrect/missing volpkg location!" << std::endl;
        exit(EXIT_FAILURE);
    }

    VolumePkg volpkg(volpkgpath);

    if( mode == "-p"){
        std::cout << "INITIAL METADATA: " << std::endl;
        volpkg.printJSON();
        std::cout << std::endl;
        return EXIT_SUCCESS;
    }
    else if ( mode == "-t" || mode == "-w" ) {
        std::map<std::string, std::string> parsedMetadata;

        // Parse the metadata key and its value
        std::cout << "Parsing arguments..." << std::endl;
        for (int i = 3; i < argc; ++i) {
            std::string argument = argv[i];
            std::size_t delimiter = argument.find("=");
            if (delimiter == argument.npos) {
                std::cerr << "\"" << argument << "\" does not match the format key=value." << std::endl;
                continue;
            }
            std::string key = argument.substr(0, delimiter);
            std::string value = argument.substr(delimiter + 1, argument.npos);

            parsedMetadata[key] = value;
        }

        std::cout << std::endl;
        if (parsedMetadata.size() == 0){
            std::cout << "No recognized key=value pairs given. Metadata will not be changed." << std::endl << std::endl;
            return EXIT_SUCCESS;
        }

        // Change the version number first since that affects everything else
        std::map<std::string, std::string>::const_iterator versionFind = parsedMetadata.find("version");
        if (versionFind != parsedMetadata.end()) {
            std::cerr << "ERROR: Version upgrading is not available at this time." << std::endl;
            /* We only have one volpkg version, so this is disabled for right now - SP, 4/30/2015
            double currentVersion = volpkg.getVersion();
            double newVersion = boost::lexical_cast<double>((*versionFind).second);
            if (currentVersion == newVersion) {
                std::cout << "Volpkg v." << currentVersion << " == v." << (*versionFind).second << std::endl;
                std::cout << "Volpkg will not be converted." << std::endl;
                parsedMetadata.erase(versionFind);
            } else {
                std::cout << "Attempting to convert volpkg from v." << currentVersion << " -> v." <<
                (*versionFind).second << std::endl;
                if (volpkg.setMetadata((*versionFind).first, (*versionFind).second) == EXIT_SUCCESS) {
                    // To-Do: Upgrade the json object to match the new dict
                    std::cout << "Version set successfully." << std::endl;
                    parsedMetadata.erase(versionFind);
                }
                else {
                    std::cerr << "ERROR: Volpkg could not be converted." << std::endl;
                    return EXIT_FAILURE;
                }
            } */
            parsedMetadata.erase((*versionFind).first);
            std::cout << std::endl;
        }

        // Attempt to set metadata keys to specified values
        std::map<std::string, std::string>::iterator parsedIterator;
        for (parsedIterator = parsedMetadata.begin(); parsedIterator != parsedMetadata.end(); ++parsedIterator) {
            std::cout << "Attempting to set key \"" << (*parsedIterator).first << "\" to value \"" <<
            (*parsedIterator).second << "\"" << std::endl;
            if (volpkg.setMetadata((*parsedIterator).first, (*parsedIterator).second) == EXIT_SUCCESS) {
                std::cout << "Key set successfully." << std::endl;
            }
            std::cout << std::endl;
        }

        if( mode == "-t"){
            std::cout << "FINAL METADATA: " << std::endl;
            volpkg.printJSON();
            std::cout << std::endl;
            return EXIT_SUCCESS;
        } else if( mode == "-w"){
            // save the new json file to test.json
            std::cout << "Writing metadata to file..." << std::endl;
            volpkg.saveMetadata();
            std::cout << "Metadata written successfully." << std::endl << std::endl;
            return EXIT_SUCCESS;
        }
    }
    else {
        std::cerr << "ERROR: Unrecognized mode." << std::endl;
        return EXIT_FAILURE;
    }
}