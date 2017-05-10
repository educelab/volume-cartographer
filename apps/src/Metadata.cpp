// VC Metadata Viewer/Editor
#include <iostream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/types/VolumePkgVersion.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace vc = volcart;

int main(int argc, char* argv[])
{
    // clang-format off
    po::options_description options{"options arguments"};
    options.add_options()
        ("help,h", "Show this message")
        ("print,p", "Print current metadata")
        ("test,t", "Test metadata changes but do not write to file")
        ("write,w", "Write metadata changes to file")
        ("volpkg,v", po::value<fs::path>()->required(), "Path to volumepkg")
        ("configs", po::value<std::vector<std::string>>(),
            "New metadata key/value pairs");
    po::positional_options_description positional;
    positional.add("configs", -1);
    // clang-format on

    po::command_line_parser parser{argc, argv};
    parser.options(options).positional(positional).allow_unregistered();
    auto parsed = parser.run();
    po::variables_map opts;
    po::store(parsed, opts);

    // Print help
    if (argc == 1 || opts.count("help")) {
        std::cout << "Usage: " << argv[0]
                  << " [options] key=value [key=value ...]" << std::endl;
        std::cout << options << std::endl;
        std::exit(1);
    }

    // Warn of missing options
    try {
        po::notify(opts);
    } catch (po::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Verify only one mode specified
    if (opts.count("test") + opts.count("write") > 1) {
        std::cerr
            << "Multiple modes specified. Only pick one of [print/test/write]"
            << std::endl;
        std::exit(1);
    }

    // Build volumepkg
    vc::VolumePkg volpkg{opts["volpkg"].as<fs::path>()};

    // Print metadata
    if (opts.count("print")) {
        std::cout << "Initial metadata: " << std::endl;
        volpkg.printJSON();
        std::cout << std::endl;
    }

    // Test or write metadata
    if (opts.count("test") || opts.count("write")) {
        if (!opts.count("configs")) {
            std::cout << "No metadata changes to make, exiting" << std::endl;
            std::exit(0);
        }
        auto configs = opts["configs"].as<std::vector<std::string>>();

        // Parse the metadata key and its value
        std::map<std::string, std::string> parsedMetadata;
        for (auto&& config : configs) {
            auto delimiter = config.find("=");
            if (delimiter == std::string::npos) {
                std::cerr << "\"" << config
                          << "\" does not match the format key=value."
                          << std::endl;
                continue;
            }
            auto key = config.substr(0, delimiter);
            auto value = config.substr(delimiter + 1, config.npos);

            parsedMetadata[key] = value;
        }

        std::cout << std::endl;
        if (parsedMetadata.empty()) {
            std::cout << "No recognized key=value pairs given. Metadata will "
                         "not be changed."
                      << std::endl;
            return EXIT_SUCCESS;
        }

        // Change the version number first since that affects everything else
        auto versionFind = parsedMetadata.find("version");
        if (versionFind != std::end(parsedMetadata)) {
            std::cerr
                << "ERROR: Version upgrading is not available at this time."
                << std::endl;
            parsedMetadata.erase(versionFind->first);
            std::cout << std::endl;
        }

        // Find metadata type mapping for given version.
        auto types_it = volcart::VERSION_LIBRARY.find(volpkg.getVersion());
        if (types_it == std::end(volcart::VERSION_LIBRARY)) {
            std::cerr << "Could not find type mapping for version "
                      << volpkg.getVersion() << std::endl;
            std::exit(1);
        }
        auto typeMap = types_it->second;

        // Parse metadata arguments.
        for (auto&& pair : parsedMetadata) {
            std::cout << "Attempting to set key \"" << pair.first
                      << "\" to value \"" << pair.second << "\"" << std::endl;

            // Find the key mapping
            auto t = typeMap.find(pair.first);
            if (t == typeMap.end()) {
                std::cout << "Key \"" << pair.first
                          << "\" not found in dictionary. Skipping."
                          << std::endl;
                continue;
            }

            switch (t->second) {
                case volcart::Type::STRING:
                    volpkg.setMetadata(pair.first, pair.second);
                    break;
                case volcart::Type::INT:
                    volpkg.setMetadata(pair.first, std::stoi(pair.second));
                    break;
                case volcart::Type::DOUBLE:
                    volpkg.setMetadata(pair.first, std::stod(pair.second));
                    break;
            }
            std::cout << std::endl;
        }

        // Only print, don't save.
        if (opts.count("test")) {
            std::cout << "Final metadata: " << std::endl;
            volpkg.printJSON();
            std::cout << std::endl;
            return EXIT_SUCCESS;
        }

        // Actually save.
        if (opts.count("write")) {
            std::cout << "Writing metadata to file..." << std::endl;
            volpkg.saveMetadata();
            std::cout << "Metadata written successfully." << std::endl
                      << std::endl;
            return EXIT_SUCCESS;
        }
    }
}
