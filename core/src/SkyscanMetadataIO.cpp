#include "vc/core/io/SkyscanMetadataIO.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "vc/core/types/Exceptions.hpp"
#include "vc/core/util/String.hpp"

using namespace volcart;

// Read and return map of metadata values
auto SkyscanMetadataIO::read() -> volcart::Metadata
{
    parse_();
    return metadata_;
}

// Parse the file
void SkyscanMetadataIO::parse_()
{
    /** Setup known metadata regexps **/
    // System
    std::regex secSystem{"^\\[System\\]"};
    std::regex scanner{"^Scanner"};
    // User
    std::regex secUser{"^\\[User\\]"};
    // Acquisition
    std::regex secAcq{"^\\[Acquisition\\]"};
    std::regex sourceVolt{"^Source Voltage"};
    std::regex sourceCurrent{"^Source Current"};
    std::regex imgPixSize{"^Image Pixel Size"};
    std::regex objToSrc{"^Object to Source"};
    std::regex camToSource{"^Camera to Source"};
    std::regex depth{"^Depth"};
    std::regex exposure{"^Exposure"};
    std::regex rotationStep{"^Rotation Step"};
    std::regex trajectory{"^Scanning Trajectory"};
    std::regex motionType{"^Type Of Motion"};
    std::regex connectedScanNum{"^Number of connected scans"};
    std::regex studyDateTime{"^Study Date and Time"};
    std::regex duration{"^Scan duration"};
    // Reconstruction
    std::regex secRecon{"^\\[Reconstruction\\]"};
    std::regex datasetPrefix{"^Dataset Prefix"};
    std::regex dateTime{"^Time and Date"};
    std::regex resultFileType{"^Result File Type"};
    std::regex resultFileHeaderLength{"^Result File Header Length"};
    std::regex resultImageWidth{"^Result Image Width"};
    std::regex resultImageHeight{"^Result Image Height"};
    std::regex voxelSize{"^Pixel Size \\(um\\)"};
    std::regex topROI{"^ROI Top"};
    std::regex bottomROI{"^ROI Bottom"};
    std::regex leftROI{"^ROI Left"};
    std::regex rightROI{"^ROI Right"};
    std::regex referenceLengthROI{"^ROI Reference Length"};
    // Naming
    std::regex secNaming{"^\\[File name convention\\]"};
    std::regex indexLength{"^Filename Index Length"};
    std::regex prefixRegex{"^Filename Prefix"};

    /** Begin log parsing */
    // Load the log
    std::ifstream ifs(path_.string());
    if (!ifs.good()) {
        throw IOException("Failed to open file for reading");
    }

    // Parse each line
    std::string line;
    std::vector<std::string> lineTokens;
    while (std::getline(ifs, line)) {
        // Parse the line
        trim(line);
        lineTokens = split(line, '=');
        std::for_each(
            std::begin(lineTokens), std::end(lineTokens),
            [](auto& s) { trim(s); });

        if (std::regex_match(lineTokens[0], scanner)) {
            metadata_.set<std::string>("scanner", lineTokens[1]);
        }

        else if (std::regex_match(lineTokens[0], objToSrc)) {
            metadata_.set<double>("objToSrc", std::stod(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], camToSource)) {
            metadata_.set<double>("camToSource", std::stod(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], sourceVolt)) {
            metadata_.set<double>("srcVoltage", std::stod(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], sourceCurrent)) {
            metadata_.set<double>("srcCurrent", std::stod(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], depth)) {
            metadata_.set<int>("depth", std::stoi(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], exposure)) {
            metadata_.set<double>("exposureTime", std::stod(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], rotationStep)) {
            metadata_.set<double>("rotationStep", std::stod(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], connectedScanNum)) {
            metadata_.set<int>("connectedScanNum", std::stoi(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], studyDateTime)) {
            metadata_.set<std::string>("studyDateTime", lineTokens[1]);
        }

        else if (std::regex_match(lineTokens[0], datasetPrefix)) {
            metadata_.set<std::string>("datasetPrefix", lineTokens[1]);
        }

        else if (std::regex_match(lineTokens[0], dateTime)) {
            metadata_.set<std::string>("dateTime", lineTokens[1]);
        }

        else if (std::regex_match(lineTokens[0], resultFileType)) {
            metadata_.set<std::string>("resultFileType", lineTokens[1]);
        }

        else if (std::regex_match(lineTokens[0], resultFileHeaderLength)) {
            metadata_.set<int>(
                "resultFileHeaderLength", std::stoi(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], resultImageWidth)) {
            metadata_.set<int>("resultImageWidth", std::stoi(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], resultImageHeight)) {
            metadata_.set<int>("resultImageHeight", std::stoi(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], voxelSize)) {
            metadata_.set<double>("voxelSize", std::stod(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], topROI)) {
            metadata_.set<int>("topROI", std::stoi(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], bottomROI)) {
            metadata_.set<int>("bottomROI", std::stoi(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], leftROI)) {
            metadata_.set<int>("leftROI", std::stoi(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], rightROI)) {
            metadata_.set<int>("rightROI", std::stoi(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], referenceLengthROI)) {
            metadata_.set<int>("referenceLengthROI", std::stoi(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], indexLength)) {
            metadata_.set<int>("indexLength", std::stoi(lineTokens[1]));
        }

        else if (std::regex_match(lineTokens[0], prefixRegex)) {
            metadata_.set<std::string>("sliceImgPrefix", lineTokens[1]);
        }

        // Clear the token string
        lineTokens.clear();
    }

    // Close the log file
    ifs.close();
}

auto SkyscanMetadataIO::getSliceRegexString() -> std::string
{
    // Get components
    auto prefix = metadata_.get<std::string>("sliceImgPrefix");
    auto idxLen = metadata_.get<int>("indexLength");
    auto format = metadata_.get<std::string>("resultFileType");

    // Build regex string
    auto regexStr = prefix + "\\d{" + std::to_string(idxLen) + "}\\.";

    // Concat file extension
    if (format == "TIF") {
        regexStr += "(tif|tiff|TIF|TIFF)";
    } else if (format == "JPG") {
        regexStr += "(jpg|jpeg|JPG|JPEG)";
    } else if (format == "PNG") {
        regexStr += "(png|PNG)";
    } else if (format == "BMP") {
        regexStr += "(bmp|BMP)";
    } else {
        regexStr += ".*";
    }

    return regexStr;
}
