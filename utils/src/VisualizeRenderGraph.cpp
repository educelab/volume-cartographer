#include <iostream>
#include <map>

#include <boost/program_options.hpp>
#include <smgl/Graphviz.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/String.hpp"
#include "vc/graph.hpp"

namespace vc = volcart;
namespace fs = volcart::filesystem;
namespace po = boost::program_options;

using namespace volcart;

static inline auto EduceLabStyle() -> smgl::GraphStyle
{
    smgl::GraphStyle style;
    style.defaultStyle().inputPorts.bgcolor = "#87BF73";
    style.defaultStyle().inputPorts.color = "#87BF73";
    style.defaultStyle().label.bgcolor = "#1897D4";
    style.defaultStyle().label.color = "#1897D4";
    style.defaultStyle().outputPorts.bgcolor = "#4C9D2F";
    style.defaultStyle().outputPorts.color = "#4C9D2F";
    style.defaultStyle().font.color = "white";
    return style;
}

const std::map<std::string, smgl::GraphStyle> STYLE_MAP{
    {"bw", smgl::GraphStyle()}, {"educelab", EduceLabStyle()}};

auto main(int argc, char* argv[]) -> int
{
    ///// Parse the command line options /////
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
    ("help,h", "Show this message")
    ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
    ("render,r", po::value<std::string>()->required(), "Render ID")
    ("style,s", po::value<std::string>()->default_value("bw"),
        "Graph style options: bw, educelab")
    ("output-file,o", po::value<std::string>(),
        "Output file path. If not specified, the file will be "
        "written to [render-id].gv");

    po::options_description all("Usage");
    all.add(required);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") > 0 || argc < 5) {
        std::cout << all << "\n";
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        vc::Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    // Register Volume Cartographer nodes
    vc::RegisterNodes();

    // Load the volume package
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();
    vc::VolumePkg vpkg(volpkgPath);

    // Get the render
    auto renderID = parsed["render"].as<std::string>();
    auto render = vpkg.render(renderID);

    // Load the render graph
    auto graph = render->graph();

    // Load the graph style
    auto styleType = parsed["style"].as<std::string>();
    vc::to_lower(styleType);
    if (STYLE_MAP.count(styleType) == 0) {
        vc::Logger()->warn(
            "Unrecognized style type: {}. Using default.", styleType);
        styleType = "bw";
    }
    const auto& style = STYLE_MAP.at(styleType);

    // Write the graphviz file
    fs::path outFile;
    if (parsed.count("output-file") > 0) {
        outFile = parsed["output-file"].as<std::string>();
    } else {
        outFile = renderID + ".gv";
    }
    smgl::WriteDotFile(outFile, *graph, style);
}