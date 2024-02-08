#include "vc/core/io/UVMapIO.hpp"

#include <cstddef>
#include <fstream>
#include <regex>
#include <sstream>

#include "vc/core/types/Exceptions.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/String.hpp"

using namespace volcart;
using namespace volcart::io;

namespace fs = volcart::filesystem;
namespace vio = volcart::io;

void vio::WriteUVMap(const fs::path& path, const UVMap& uvMap)
{
    std::ofstream outfile{path.string(), std::ios::binary};
    if (!outfile.is_open()) {
        auto msg = "could not open file '" + path.string() + "'";
        throw IOException(msg);
    }

    // Header
    std::stringstream ss;
    ss << "filetype: uvmap" << '\n';
    ss << "version: 1" << '\n';
    ss << "type: per-vertex" << '\n';
    ss << "size: " << uvMap.size() << '\n';
    ss << "width: " << uvMap.ratio().width << '\n';
    ss << "height: " << uvMap.ratio().height << '\n';
    ss << "origin: " << static_cast<int>(uvMap.origin()) << '\n';
    ss << "<>" << '\n';
    outfile << ss.rdbuf();

    // Write the mappings
    for (const auto& mapping : uvMap.as_map()) {
        const auto& id = mapping.first;
        const auto& uv = mapping.second;
        outfile.write(reinterpret_cast<const char*>(&id), sizeof(id));
        auto nbytes = 2 * sizeof(double);
        outfile.write(reinterpret_cast<const char*>(uv.val), nbytes);
    }

    outfile.close();
}

auto vio::ReadUVMap(const fs::path& path) -> UVMap
{
    std::ifstream infile{path.string(), std::ios::binary};
    if (!infile.is_open()) {
        auto msg = "could not open file '" + path.string() + "'";
        throw IOException(msg);
    }

    struct Header {
        std::string fileType;
        int version;
        std::string type;
        std::size_t size;
        double width{0};
        double height{0};
        int origin{-1};
    };

    // Regexes
    std::regex comments{"^#"};
    std::regex fileType{"^filetype"};
    std::regex version{"^version"};
    std::regex type{"^type"};
    std::regex size{"^size"};
    std::regex width{"^width"};
    std::regex height{"^height"};
    std::regex origin{"^origin"};
    std::regex headerTerminator{"^<>$"};

    Header h;
    std::string line;
    while (std::getline(infile, line)) {
        trim(line);
        auto strs = split(line, ':');
        std::for_each(
            std::begin(strs), std::end(strs), [](auto& s) { trim(s); });

        // Comments: look like:
        // # This is a comment
        //    # This is another comment
        if (std::regex_match(strs[0], comments)) {
            continue;
        }

        // File type
        else if (std::regex_match(strs[0], fileType)) {
            if (strs[1] != "uvmap") {
                throw IOException("File mismatch. File is not UVMap.");
            }
            h.fileType = strs[1];
        }

        // Version
        else if (std::regex_match(strs[0], version)) {
            auto fileVersion = std::stoi(strs[1]);
            if (fileVersion != 1) {
                auto msg = "Version mismatch. UVMap file version is " +
                           strs[1] + ", processing version is 1.";
                throw IOException(msg);
            }
        }

        // Type
        else if (std::regex_match(strs[0], type)) {
            if (strs[1] != "per-vertex") {
                throw IOException("UVMap mismatch. File is not per-vertex.");
            }
            h.type = strs[1];
        }

        // Size
        else if (std::regex_match(strs[0], size)) {
            h.size = std::stoul(strs[1]);
        }

        // Width
        else if (std::regex_match(strs[0], width)) {
            h.width = std::stod(strs[1]);
        }

        // Height
        else if (std::regex_match(strs[0], height)) {
            h.height = std::stod(strs[1]);
        }

        // Origin
        else if (std::regex_match(strs[0], origin)) {
            h.origin = std::stoi(strs[1]);
        }

        // End of the header
        else if (std::regex_match(line, headerTerminator)) {
            break;
        }

        // Ignore everything else
        else {
            continue;
        }
        strs.clear();
    }

    // Sanity check. Do we have a valid UVMap header?
    if (h.fileType.empty()) {
        const auto* msg = "Must provide file type";
        throw IOException(msg);
    } else if (h.type.empty()) {
        auto msg = "Must provide type";
        throw IOException(msg);
    } else if (h.width == 0 or h.height == 0) {
        throw IOException("UVMap cannot have dimensions == 0");
    } else if (h.origin == -1) {
        throw IOException("UVMap file does not contain origin");
    }

    // Construct the UVMap
    UVMap map;
    map.setOrigin(static_cast<UVMap::Origin>(h.origin));
    map.ratio(h.width, h.height);

    // Read all of the points
    for (const auto& i : range(h.size)) {
        std::ignore = i;
        std::size_t id{0};
        cv::Vec2d uv;
        infile.read(reinterpret_cast<char*>(&id), sizeof(id));
        infile.read(reinterpret_cast<char*>(uv.val), 2 * sizeof(double));
        map.set(id, uv);
    }

    return map;
}
