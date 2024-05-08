#include <cstddef>
#include <iostream>

#include <boost/program_options.hpp>

#include <QCoreApplication>

#include "vc/app_support/GetMemorySize.hpp"
#include "vc/apps/server/VolumeProtocol.hpp"
#include "vc/apps/server/VolumeServer.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/neighborhood/CuboidGenerator.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/MemorySizeStringParser.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

// Volpkg version required by this app
static constexpr int VOLPKG_MIN_VERSION = 6;

auto main(int argc, char* argv[]) -> int
{
    // Add CLI arguments
    // clang-format off
    std::vector<std::string> volpkgPaths;
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("port,p", po::value<quint16>()->default_value(8087), "Port to listen on")
        ("memory,m", po::value<std::string>()->required(), "Memory to reserve for the server in bytes (accepts K, M, G, T suffixes)")
        ("volpkg,v", po::value(&volpkgPaths)->multitoken()->required(), "VolumePkg path (required, repeatable option)");

    po::options_description all("Usage");
    all.add(required);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") || argc < 2) {
        std::cout << all << '\n';
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        vc::Logger()->error("{}", e.what());
        return EXIT_FAILURE;
    }

    // Get the port to listen on
    quint16 port = parsed["port"].as<quint16>();

    // Get the memory to reserve
    std::string memoryString = parsed["memory"].as<std::string>();
    std::size_t memory = 0;
    try {
        memory = vc::MemorySizeStringParser(memoryString);
        if (memory > SystemMemorySize()) {
            memory = SystemMemorySize();
        }
        if (memory < 1) {
            throw std::domain_error("Memory must be positive");
        }
    } catch (std::domain_error& e) {
        vc::Logger()->error("{}", e.what());
        return EXIT_FAILURE;
    }
    vc::Logger()->info(
        "Server will use no more than {} bytes of memory for volumes.", memory);

    // Load the volume packages
    vc::VolumeServer::VolumePkgMap volpkgs;
    for (auto volpkgPath : volpkgPaths) {
        vc::Logger()->info("Loading volume package: {}", volpkgPath);
        try {
            vc::VolumePkg volpkg(volpkgPath);
            if (volpkg.version() < VOLPKG_MIN_VERSION) {
                vc::Logger()->error(
                    "Volume package is version {} but this program requires "
                    "version {}.",
                    volpkg.version(), VOLPKG_MIN_VERSION);
                return EXIT_FAILURE;
            }
            if (volpkgs.count(volpkg.name())) {
                vc::Logger()->error(
                    "Tried to load volume package with name {} but a volume "
                    "package with that name is already loaded.",
                    volpkg.name());
                return EXIT_FAILURE;
            }
            volpkgs.insert({volpkg.name(), volpkg});
        } catch (std::exception& e) {
            vc::Logger()->error(
                "Failed to load volume package: {}: {}", volpkgPath, e.what());
            return EXIT_FAILURE;
        }
    }

    // Start the QtCoreApplication
    QCoreApplication application(argc, argv);
    vc::VolumeServer server(volpkgs, port, memory);
    QObject::connect(
        &server, &vc::VolumeServer::finished, &application,
        &QCoreApplication::quit);
    return application.exec();
}
