#include <cstring>
#include <iostream>

#include <boost/program_options.hpp>

#include <QCoreApplication>

#include "vc/apps/server/VolumeClient.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/util/Logging.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

int main(int argc, char* argv[])
{
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("server,s", po::value<std::string>()->required(), "IP address of the Volume Server")
        ("port,p", po::value<quint16>()->required(), "Port of the Volume Server");

    po::options_description all("Usage");
    all.add(required);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") || argc < 5) {
        std::cout << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        vc::Logger()->error("{}", e.what());
        return EXIT_FAILURE;
    }

    // Get the parsed options
    std::string server_ip = parsed["server"].as<std::string>();
    quint16 server_port = parsed["port"].as<quint16>();

    // Launch the Qt CLI application
    QCoreApplication application(argc, argv);
    vc::VolumeClient client_(QString::fromStdString(server_ip), server_port);
    QObject::connect(
        &client_, &vc::VolumeClient::finished, &application,
        &QCoreApplication::quit);
    return application.exec();
}
