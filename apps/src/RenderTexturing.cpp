#include "vc/apps/render/RenderTexturing.hpp"

#include <cstdint>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

auto GetFilteringOpts() -> po::options_description
{
    // clang-format off
    po::options_description opts("Generic Texture Filtering Options");
    opts.add_options()
        ("method,m", po::value<int>()->default_value(0),
             "Texturing method: \n"
                 "  0 = Composite\n"
                 "  1 = Intersection\n"
                 "  2 = Integral\n"
                 "  3 = Thickness")
        ("neighborhood-shape,n", po::value<int>()->default_value(0),
             "Neighborhood shape:\n"
                 "  0 = Linear\n"
                 "  1 = Cuboid")
        ("radius,r", po::value<std::vector<double>>()->multitoken(), "Search "
            "radius. Defaults to value calculated from estimated layer "
            "thickness.")
        ("interval,i", po::value<double>()->default_value(1.0),
            "Sampling interval")
        ("direction,d", po::value<int>()->default_value(0),
            "Sample Direction:\n"
                " -1 = Negative\n"
                "  0 = Omni\n"
                "  1 = Positive")
        ("shading", po::value<int>()->default_value(1),
            "Surface Normal Shading:\n"
                "  0 = Flat\n"
                "  1 = Smooth");
    // clang-format on

    return opts;
}

auto GetCompositeOpts() -> po::options_description
{
    // clang-format off
    po::options_description opts("Composite Texture Options");
    opts.add_options()
        ("filter,f", po::value<int>()->default_value(1),
            "Filter:\n"
                "  0 = Minimum\n"
                "  1 = Maximum\n"
                "  2 = Median\n"
                "  3 = Mean\n"
                "  4 = Median w/ Averaging");
    // clang-format on

    return opts;
}

auto GetIntegralOpts() -> po::options_description
{
    // clang-format off
    po::options_description opts("Integral Texture Options");
    opts.add_options()
        ("weight-type,w", po::value<int>()->default_value(0),
            "Weight Type:\n"
                "  0 = None\n"
                "  1 = Linear\n"
                "  2 = Exponential Difference")
        ("linear-weight-direction", po::value<int>()->default_value(0),
            "Linear Weight Direction:\n"
                "  0 = Favor the + normal direction\n"
                "  1 = Favor the - normal direction")
        ("expodiff-exponent", po::value<int>()->default_value(2), "Exponent "
            "applied to the absolute difference values.")
        ("expodiff-base-method", po::value<int>()->default_value(0),
            "Exponential Difference Base Calculation Method:\n"
                "  0 = Mean\n"
                "  1 = Mode\n"
                "  2 = Manually specified")
        ("expodiff-base", po::value<double>()->default_value(0.0), "If the "
            "base calculation method is set to Manual, the value from which "
            "voxel values are differenced.")
        ("clamp-to-max", po::value<std::uint16_t>(), "Clamp values to the specified "
            "maximum.");
    // clang-format on

    return opts;
}

auto GetThicknessOpts() -> po::options_description
{
    // clang-format off
    po::options_description opts("Thickness Texture Options");
    opts.add_options()
        ("volume-mask", po::value<std::string>(),
            "Path to volumetric mask point set")
        ("normalize-output", po::value<bool>()->default_value(true),
            "Normalize the output image between [0, 1]. If enabled "
            "(default), the output file should be a TIFF file and the "
            "--tiff-floating-point flag should be provided.");
    // clang-format on

    return opts;
}
