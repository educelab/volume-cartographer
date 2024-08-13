#include "vc/core/types/Annotation.hpp"

#include <fstream>
#include <optional>
#include <regex>

#include <educelab/core/utils/Iteration.hpp>
#include <educelab/core/utils/String.hpp>
#include <sys/stat.h>

#include "vc/core/util/Logging.hpp"

using namespace volcart;
namespace el = educelab;

namespace
{
struct Header {
    std::optional<std::string> fileType;
    std::optional<int> version;
    std::size_t width{};
    std::size_t height{};
    std::string type{};
    bool ordered{false};
};

auto ParseHeader(std::ifstream& infile) -> Header
{
    Header h;
    std::string line;
    while (std::getline(infile, line)) {
        auto l = el::trim(line);
        auto strs = el::split(l, ":");
        std::for_each(std::begin(strs), std::end(strs), [](auto& s) {
            return el::trim(s);
        });

        // Comments: look like:
        // # This is a comment
        //    # This is another comment
        if (strs[0].front() == '#') {
            continue;
        }

        // Version
        if (strs[0] == "class") {
            h.fileType = strs[1];
        }

        // Version
        else if (strs[0] == "version") {
            h.version = el::to_numeric<int>(strs[1]);
        }

        // Width
        else if (strs[0] == "width") {
            h.width = el::to_numeric<std::size_t>(strs[1]);
        }

        // Height
        else if (strs[0] == "height") {
            h.height = el::to_numeric<std::size_t>(strs[1]);
        }

        // Height
        else if (strs[0] == "type") {
            h.type = strs[1];
        }

        // Ordering
        else if (strs[0] == "ordered") {
            const auto ordered = el::to_lower(std::string(strs[1]));
            if (ordered == "true") {
                h.ordered = true;
            } else if (ordered == "false") {
                h.ordered = false;
            } else {
                Logger()->warn("Unrecognized value for ordered: {}", strs[1]);
            }
        }

        // End-of-header
        else if (strs[0] == "<>") {
            break;
        }
    }

    return h;
}
}  // namespace

Annotation::Annotation(
    const std::int32_t i,
    const AnnotationFlags f,
    const double x,
    const double y)
    : index{i}, flags{f}, pt{x, y}
{
}

void volcart::WriteAnnotationSet(
    const filesystem::path& path, const AnnotationSet& as)
{
    std::ofstream of{path.string(), std::ios::binary};
    if (not of.is_open()) {
        auto msg = "could not open file '" + path.string() + "'";
        throw IOException(msg);
    }

    // Write header
    std::stringstream ss;
    ss << "class: annotation" << std::endl;
    ss << "version: 1" << std::endl;
    ss << "width: " << as.width() << std::endl;
    ss << "height: " << as.height() << std::endl;
    ss << "type: int32,int32,double,double" << std::endl;
    ss << "ordered: true" << std::endl;
    ss << "<>" << std::endl;
    auto header = ss.str();
    of.write(header.c_str(), static_cast<std::streamsize>(header.size()));

    // Write the annotation points
    // TODO: Probably could write all values at once
    for (const auto& a : as) {
        of.write(reinterpret_cast<const char*>(&a.index), sizeof(a.index));
        of.write(reinterpret_cast<const char*>(&a.flags), sizeof(a.flags));
        of.write(reinterpret_cast<const char*>(a.pt.val), sizeof(double) * 2);
    }

    of.flush();
    of.close();
    if (of.fail()) {
        const auto msg = "failure writing file '" + path.string() + "'";
        throw IOException(msg);
    }
}

auto volcart::ReadAnnotationSet(const filesystem::path& path) -> AnnotationSet
{
    std::ifstream infile{path.string(), std::ios::binary};
    if (!infile.is_open()) {
        const auto msg = "could not open file '" + path.string() + "'";
        throw IOException(msg);
    }

    // Parse the header
    auto header = ParseHeader(infile);

    // Check if this file was written with PointSetIO
    auto isLegacy = not header.fileType.has_value();

    // Initialize the AnnotationSet
    AnnotationSet as{header.width};

    // Have to go point-by-point
    // TODO: If not legacy, could probably read all row points in one call
    std::vector<Annotation> row;
    Annotation tmp;
    double idx, flags;
    row.reserve(header.width);
    for (const auto _r : el::range(header.height)) {
        row.clear();
        for (const auto _c : el::range(header.width)) {
            if (isLegacy) {
                infile.read(reinterpret_cast<char*>(&idx), sizeof(double));
                infile.read(reinterpret_cast<char*>(&flags), sizeof(double));
                tmp.index = static_cast<std::int32_t>(idx);
                tmp.flags = static_cast<AnnotationFlags>(flags);
            } else {
                infile.read(
                    reinterpret_cast<char*>(&tmp.index), sizeof(tmp.index));
                infile.read(
                    reinterpret_cast<char*>(&tmp.flags), sizeof(tmp.flags));
            }
            infile.read(
                reinterpret_cast<char*>(tmp.pt.val), sizeof(double) * 2);
            row.emplace_back(tmp);
        }
        as.pushRow(row);
    }

    return as;
}