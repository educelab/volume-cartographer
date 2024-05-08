#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <boost/program_options.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

auto GetValue(int x, int y, const cv::Mat& m) -> float;

auto main(int argc, char* argv[]) -> int
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description all("Usage");
    all.add_options()
        ("help,h", "Show this message")
        ("input,i", po::value<std::string>()->required(),
             "Path to a texture image.")
        ("output,o", po::value<std::string>()->required(),
             "Path to an output CSV file.")
        ("mask,m", po::value<std::string>(),
             "Path to the mask file.")
        ("class,c", po::value<std::vector<std::string>>(), "Path to a class mask file.");
    // clang-format on

    // parsed will hold the values of all parsed options as a Map
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
        vc::Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    // Get paths
    fs::path inputPath = parsed["input"].as<std::string>();
    fs::path outputPath = parsed["output"].as<std::string>();

    // Load the input image
    auto img = cv::imread(inputPath.string(), cv::IMREAD_UNCHANGED);
    std::vector<float> allVals;

    // Load mask
    cv::Mat mask;
    if (parsed.count("mask") > 0) {
        auto maskPath = parsed["mask"].as<std::string>();
        mask = cv::imread(maskPath, cv::IMREAD_GRAYSCALE);
    }

    // Load the class masks
    std::vector<std::string> classNames{"all"};
    std::vector<cv::Mat> classMasks;
    std::vector<std::vector<float>> classVals;
    for (const auto& p : parsed["class"].as<std::vector<std::string>>()) {
        auto m = cv::imread(p, cv::IMREAD_GRAYSCALE);
        if (m.empty()) {
            continue;
        }
        classNames.push_back(fs::path(p).stem().string());
        classMasks.push_back(m);
        classVals.emplace_back();
    }

    // Iterate over the image pixels
    using vc::ProgressWrap;
    using vc::range2D;
    for (const auto pixel :
         ProgressWrap(range2D(img.rows, img.cols), "Collecting values")) {
        const auto& x = pixel.second;
        const auto& y = pixel.first;

        // Skip if we have a mask and this pixel isn't in it
        if (not mask.empty() and mask.at<std::uint8_t>(y, x) == 0) {
            continue;
        }

        // Get the pixel value
        auto val = GetValue(x, y, img);

        // Add to the all values list
        allVals.push_back(val);

        // Add to the class lists
        using vc::enumerate;
        for (const auto it : enumerate(classMasks)) {
            const auto& idx = it.first;
            const auto& cmask = it.second;
            if (cmask.at<std::uint8_t>(y, x) > 0) {
                classVals[idx].push_back(val);
            }
        }
    }

    // Calculate metrics
    classVals.insert(classVals.begin(), allVals);
    using MetricMap = std::map<std::string, float>;
    std::vector<MetricMap> metrics;
    for (auto& c : classVals) {
        MetricMap m;

        // Sample count
        m["count"] = c.size();

        // Mean
        auto mean = std::accumulate(
            c.begin(), c.end(), 0.F,
            [size = float(c.size())](float r, float v) {
                return r + (v / size);
            });
        m["mean"] = mean;

        // Variance
        auto var =
            std::accumulate(
                c.begin(), c.end(), 0.F,
                [mean](auto c, auto r) { return c + std::pow(r - mean, 2); }) /
            c.size();
        m["var"] = var;

        // Std. deviation
        m["std_dev"] = std::sqrt(var);

        // Median
        std::nth_element(c.begin(), c.begin() + c.size() / 2, c.end());
        m["median"] = c[c.size() / 2];

        metrics.push_back(m);
    }

    // Write metrics
    std::ofstream file(outputPath.string());
    if (not file.is_open()) {
        throw std::runtime_error("Couldn't open output file");
    }
    file << "class,count,mean,var,std_dev,median" << '\n';
    for (const auto it : vc::enumerate(metrics)) {
        file << classNames[it.first] << ',';
        file << static_cast<std::size_t>(it.second.at("count")) << ',';
        file << it.second.at("mean") << ',';
        file << it.second.at("var") << ',';
        file << it.second.at("std_dev") << ',';
        file << it.second.at("median") << '\n';
    }
    file.close();
}

auto GetValue(int x, int y, const cv::Mat& m) -> float
{
    switch (m.depth()) {
        case CV_8U:
            return m.at<std::uint8_t>(y, x);
        case CV_8S:
            return m.at<std::int8_t>(y, x);
        case CV_16U:
            return m.at<std::uint16_t>(y, x);
        case CV_16S:
            return m.at<std::int16_t>(y, x);
        case CV_32S:
            return m.at<std::int32_t>(y, x);
        case CV_32F:
            return m.at<float>(y, x);
        case CV_64F:
            return m.at<double>(y, x);
        default:
            throw std::invalid_argument("Unsupported image depth");
    }
}