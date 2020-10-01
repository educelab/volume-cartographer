#include "vc/app_support/GeneralOptions.hpp"

namespace po = boost::program_options;

po::options_description GetGeneralOpts()
{
    // clang-format off
    po::options_description opts("General Options");
    opts.add_options()
        ("help,h", "Show this message")
        ("cache-memory-limit", po::value<std::string>(), "Maximum size of the "
            "slice cache in bytes. Accepts the suffixes: (K|M|G|T)(B). "
            "Default: 50% of the total system memory.")
        ("show-progress-bars", po::value<bool>()->default_value(true),
            "When enabled, show algorithm progress bars.");
    // clang-format on

    return opts;
}