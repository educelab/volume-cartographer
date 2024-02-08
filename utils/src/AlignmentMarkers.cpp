#include <cstddef>
#include <exception>
#include <iostream>
#include <regex>

#include <boost/program_options.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/types/TexturedMesh.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/texturing/AlignmentMarkerGenerator.hpp"

namespace po = boost::program_options;
namespace fs = volcart::filesystem;
namespace vc = volcart;
namespace vct = volcart::texturing;

vct::AlignmentMarkerGenerator::LineSegment ParseLineSegString(std::string s);

int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("input-mesh,i", po::value<std::vector<std::string>>()->required(),
             "OBJ mesh to project onto the volume. Can be specified "
             "multiple times.")
        ("line-seg,l", po::value<std::vector<std::string>>()->required(),
             "Comma-separated list of components defining the 3D "
             "start and end points of an alignment line segment: "
             "\"Sx,Sy,Sz,Ex,Ey,Ez\". A marker will be placed at the "
             "intersection points of this segment and any input meshes. Can be "
             "specified multiple times.")
        ("output-dir,o", po::value<std::string>(),
             "Output directory. If not specified, files will be "
             "written to the current working directory.")
        ("write-images-only", "Only write the texture images "
             "to disk.");

    po::options_description extraOptions("Extra Options");
    extraOptions.add_options()
        ("seg-perc-buffer", po::value<double>()->default_value(0.05),
            "Adds % more length to both ends of input line segments")
        ("marker-radius", po::value<int>()->default_value(5),
            "Marker radius (in pixels)");

    po::options_description all("Usage");
    all.add(required).add(extraOptions);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") || argc < 3) {
        std::cout << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        vc::Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    // Get options
    auto meshPaths = parsed["input-mesh"].as<std::vector<std::string>>();
    auto lineStrings = parsed["line-seg"].as<std::vector<std::string>>();
    fs::path outputDir;
    if (parsed.count("output-dir")) {
        outputDir = parsed["output-dir"].as<std::string>();
    }
    auto segBuffer = parsed["seg-perc-buffer"].as<double>();
    auto radius = parsed["marker-radius"].as<int>();

    // Get meshes
    std::cout << "Loading meshes..." << std::endl;
    vc::io::OBJReader reader;
    std::vector<vc::TexturedMesh> meshes;
    for (const auto& p : meshPaths) {
        reader.setPath(p);
        auto mesh = reader.read();
        auto uv = reader.getUVMap();
        auto img = reader.getTextureMat();
        if (not uv or uv->empty() or img.empty()) {
            vc::Logger()->warn(
                "Mesh missing UV map and/or texture image: {}", p);
            continue;
        } else {
            meshes.emplace_back(mesh, uv, img);
        }
    }

    // Setup manually specified line segments
    std::vector<vct::AlignmentMarkerGenerator::LineSegment> lineSegs;
    for (const auto& ls : lineStrings) {
        vct::AlignmentMarkerGenerator::LineSegment r;
        try {
            r = ParseLineSegString(ls);
        } catch (const std::exception& e) {
            vc::Logger()->error(
                "Could not parse string as line segment: {}", ls);
            return EXIT_FAILURE;
        }
        auto ab = cv::normalize(r.b - r.a);
        r.a += -segBuffer * ab;
        r.b += segBuffer * ab;
        lineSegs.push_back(r);
    }

    // Run marker generator
    vc::Logger()->info("Generating alignment markers...");
    vct::AlignmentMarkerGenerator marker;
    marker.setInputMeshes(meshes);
    marker.setMarkerRadius(radius);
    marker.setLineSegments(lineSegs);
    auto imgs = marker.compute();

    // Save all of the images
    vc::Logger()->info("Saving alignment marked outputs...");
    for (std::size_t idx = 0; idx < imgs.size(); idx++) {
        // Get the file name
        auto path = fs::path(meshPaths[idx]).filename();

        // Prepend the output directory if we have it
        if (!outputDir.empty()) {
            path = outputDir / (path.stem().string() + "_marked.obj");
        } else {
            path = path.stem().string() + "_marked.obj";
        }

        // Write the outputs
        if (parsed.count("write-images-only")) {
            path.replace_extension("png");
            cv::imwrite(path.string(), imgs[idx]);
        } else {
            vc::io::OBJWriter writer;
            writer.setPath(path);
            writer.setMesh(meshes[idx].mesh);
            writer.setUVMap(meshes[idx].uv);
            writer.setTexture(imgs[idx]);
            writer.write();
        }
    }

    return EXIT_SUCCESS;
}

vct::AlignmentMarkerGenerator::LineSegment ParseLineSegString(std::string s)
{
    // Parse the string into doubles
    std::regex delim(",");
    std::sregex_token_iterator it(s.begin(), s.end(), delim, -1);
    std::vector<double> components;
    for (; it != std::sregex_token_iterator(); it++) {
        components.emplace_back(std::stod(*it));
    }

    // Size check
    if (components.size() != 6) {
        throw std::invalid_argument(
            "String contains too few components to construct line segment");
    }

    // Convert to line segment
    return {
        {components[0], components[1], components[2]},
        {components[3], components[4]},
        components[5]};
}
