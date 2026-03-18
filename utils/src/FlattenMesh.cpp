#include <iostream>

#include <Eigen/Eigen>
#include <boost/program_options.hpp>
#include <educelab/core/utils/String.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/io/MeshIO.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vct = volcart::texturing;
namespace el = educelab;

using Solver = vct::AngleBasedFlattening::Solver;

auto main(int argc, char** argv) -> int
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("input-mesh,i", po::value<std::string>()->required(),
            "Input mesh file")
        ("output-mesh,o", po::value<std::string>()->required(),
            "Output mesh file")
        ("method,m", po::value<std::string>()->default_value("ABF"),
            "Flattening method: [ABF, ABF-HLSCM, LSCM, HLSCM]")
        ("solver,s", po::value<std::string>()->default_value("SparseLU"),
            "Numerical solver method (ignored for HLSCM methods): [SparseLU, CG]")
        ("threads,t", po::value<int>()->default_value(0), "Maximum number of threads")
        ("log-level", po::value<std::string>()->default_value("info"),
             "Options: off, critical, error, warn, info, debug");

    po::options_description all("Usage");
    all.add(required);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") > 0 || argc < 2) {
        std::cout << all << '\n';
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    // Set logging level
    auto logLevel = parsed["log-level"].as<std::string>();
    vc::logging::SetLogLevel(logLevel);

    // Get the method
    bool useABF{true};
    bool useHLSCM{false};
    auto method = el::to_lower_copy(parsed["method"].as<std::string>());
    if (method == "abf") {
        useABF = true;
        useHLSCM = false;
    } else if (method == "abf-hlscm") {
        useABF = true;
        useHLSCM = true;
    } else if (method == "lscm") {
        useABF = false;
        useHLSCM = false;
    } else if (method == "hlscm") {
        useABF = false;
        useHLSCM = true;
    } else {
        std::cerr << "ERROR: Unknown flattening method: " << method;
        std::cerr << '\n';
        return EXIT_FAILURE;
    }

    // Get the solver
    auto solver = Solver::SparseLU;
    const auto solverStr =
        el::to_lower_copy(parsed["solver"].as<std::string>());
    if (solverStr == "cg") {
        solver = Solver::ConjugateGradient;
    } else if (solverStr != "sparselu") {
        std::cerr << "ERROR: Unknown solver: " << solverStr << '\n';
        return EXIT_FAILURE;
    }

    // Set the number of threads (OpenMP only)
    auto threads = parsed["threads"].as<int>();
    Eigen::setNbThreads(threads);
    vc::Logger()->debug(
        "Requested threads: {}, actual threads: {}", threads,
        Eigen::nbThreads());

    // Load mesh
    vc::Logger()->info("Loading mesh...");
    fs::path inputPath = parsed["input-mesh"].as<std::string>();
    auto [mesh, uv, texture] = vc::ReadMesh(inputPath);
    vc::Logger()->info(
        "Mesh Loaded || Vertices: {} || Faces: {}", mesh->GetNumberOfPoints(),
        mesh->GetNumberOfCells());

    // Run ABF
    vct::AngleBasedFlattening abf;
    abf.setUseABF(useABF);
    abf.setUseHLSCM(useHLSCM);
    abf.setSolver(solver);
    abf.setMesh(mesh);
    mesh = abf.compute();

    vc::Logger()->info("Writing mesh...");
    fs::path outputPath = parsed["output-mesh"].as<std::string>();
    vc::WriteMesh(outputPath, mesh, uv, texture);
}