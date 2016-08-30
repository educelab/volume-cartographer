#pragma once

#include "common/types/Exceptions.h"
#include "common/types/PointSet.h"
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace volcart
{

// I/O modes
enum class IOMode { ASCII, BINARY };

template <typename T>
class PointSetIO
{
public:
    using Point = T;

    // Header definition
    struct Header {
        size_t width;
        size_t height;
        size_t size;
        size_t dim;
        bool ordered;
        std::string type;

        Header() : width(0), height(0), size(0), dim(0), ordered(false), type()
        {
        }
    };

    static PointSet<T> readFile(boost::filesystem::path path,
                                IOMode mode = IOMode::BINARY)
    {
        switch (mode) {
        case IOMode::BINARY:
            return readFileBinary(path);
        case IOMode::ASCII:
            return readFileAscii(path);
        }
    }

    static void writeFile(boost::filesystem::path path,
                          const PointSet<T>& ps,
                          IOMode mode = IOMode::BINARY)
    {
        switch (mode) {
        case IOMode::BINARY:
            return writeFileBinary(path, ps);
        case IOMode::ASCII:
            return writeFileAscii(path, ps);
        }
    }

    static std::string makeHeader(PointSet<T> ps)
    {
        std::stringstream ss;
        if (ps.isOrdered()) {
            ss << "width: " << ps.width() << std::endl;
            ss << "height: " << ps.height() << std::endl;
        } else {
            ss << "size: " << ps.capacity() << std::endl;
        }
        ss << "dim: " << Point::dim << std::endl;
        ss << "ordered: " << (ps.isOrdered() ? "true" : "false") << std::endl;

        // Output type information
        ss << "type: ";
        if (std::is_same<typename Point::Element, int>::value) {
            ss << "int" << std::endl;
        } else if (std::is_same<typename Point::Element, float>::value) {
            ss << "float" << std::endl;
        } else if (std::is_same<typename Point::Element, double>::value) {
            ss << "double" << std::endl;
        } else {
            auto msg = "unsupported type";
            throw IOException(msg);
        }

        ss << "version: " << PointSet<T>::FORMAT_VERSION << std::endl;
        ss << PointSet<T>::HEADER_TERMINATOR << std::endl;

        return ss.str();
    }

    // Note: assumes infile is already open
    static Header parseHeader(std::ifstream& infile)
    {
        // Regexes
        std::regex comments{"^#"};
        std::regex width{"^width"};
        std::regex height{"^height"};
        std::regex dim{"^dim"};
        std::regex ordering{"^ordered"};
        std::regex type{"^type"};
        std::regex size{"^size"};
        std::regex version{"^version"};
        std::regex validTypes{"^(float)|(double)|(int)"};
        std::regex headerTerminator{PointSet<T>::HEADER_TERMINATOR_REGEX};

        Header h;
        std::string line;
        std::vector<std::string> strs;

        while (std::getline(infile, line)) {
            boost::trim(line);
            boost::split(strs, line, boost::is_any_of(":"));
            std::for_each(std::begin(strs), std::end(strs),
                          [](std::string& t) { boost::trim(t); });

            // Comments: look like:
            // # This is a comment
            //    # This is another comment
            if (std::regex_match(strs[0], comments)) {
                continue;
            }

            // Width
            else if (std::regex_match(strs[0], width)) {
                h.width = std::stoul(strs[1]);
            }

            // Height
            else if (std::regex_match(strs[0], height)) {
                h.height = std::stoul(strs[1]);
            }

            // Size
            else if (std::regex_match(strs[0], size)) {
                h.size = std::stoul(strs[1]);
            }

            // Dim
            else if (std::regex_match(strs[0], dim)) {
                auto dim = std::stoul(strs[1]);
                if (dim != T::dim) {
                    auto msg =
                        "Incorrect dimension read for template specification";
                    throw IOException(msg);
                }
                h.dim = dim;
            }

            // Ordering
            else if (std::regex_match(strs[0], ordering)) {
                boost::algorithm::to_lower(strs[1]);
                if (strs[1] == "true") {
                    h.ordered = true;
                } else if (strs[1] == "false") {
                    h.ordered = false;
                } else {
                    auto msg = "'ordered' key must have value 'true'/'false'";
                    throw IOException(msg);
                }
            }

            // Type
            else if (std::regex_match(strs[0], type)) {
                if (!std::regex_match(strs[1], validTypes)) {
                    auto msg = "Valid types are int, float, double. Got: '" +
                               strs[1] + "'";
                    throw IOException(msg);
                }

                // Type validation
                auto msg = "Type mismatch: vcps filetype '" + strs[1] +
                           "' not compatible with reader type '";
                if (strs[1] == "int" &&
                    !std::is_same<typename Point::Element, int>::value) {
                    msg += "int'";
                    throw IOException(msg);
                } else if (strs[1] == "float" &&
                           !std::is_same<typename Point::Element,
                                         float>::value) {
                    msg += "float'";
                    throw IOException(msg);
                } else if (strs[1] == "double" &&
                           !std::is_same<typename Point::Element,
                                         double>::value) {
                    msg += "double'";
                    throw IOException(msg);
                }
                h.type = strs[1];
            }

            // Version
            else if (std::regex_match(strs[0], version)) {
                auto fileVersion = std::stoi(strs[1]);
                if (fileVersion != PointSet<T>::FORMAT_VERSION) {
                    auto msg = "Version mismatch. VCPS file version is " +
                               strs[1] + ", processing version is " +
                               std::to_string(PointSet<T>::FORMAT_VERSION) +
                               ".";
                    throw IOException(msg);
                }
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

        // Sanity check. Do we have a valid pointset header?
        if (h.type == "") {
            auto msg = "Must provide type";
            throw IOException(msg);
        } else if (h.dim == 0) {
            auto msg = "Msut provide dim";
            throw IOException(msg);
        } else if (h.ordered == false && h.size == 0) {
            auto msg = "Unordered pointsets must have a size";
            throw IOException(msg);
        } else if (h.ordered == true && (h.width == 0 || h.height == 0)) {
            auto msg = "Ordered pointsets must have a nonzero width and height";
            throw IOException(msg);
        }

        return h;
    }

private:
    static PointSet<T> readFileAscii(boost::filesystem::path path)
    {
        std::ifstream infile{path.string()};
        if (!infile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw IOException(msg);
        }

        // Get header
        auto header = PointSetIO<T>::parseHeader(infile);

        // Ugh gross
        size_t psSize =
            (header.ordered ? header.width * header.height : header.size);
        PointSet<T> ps{psSize};
        if (header.ordered) {
            ps.setOrdered(header.width, header.height);
        }

        T tmp;
        for (size_t i = 0; i < ps.capacity(); ++i) {
            infile >> tmp;
            ps.push_back(tmp);
        }

        return ps;
    }

    static PointSet<T> readFileBinary(boost::filesystem::path path)
    {
        std::ifstream infile{path.string(), std::ios::binary};
        if (!infile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw IOException(msg);
        }
        auto header = PointSetIO<T>::parseHeader(infile);

        // Ugh gross
        size_t psSize =
            (header.ordered ? header.width * header.height : header.size);
        PointSet<T> ps{psSize};
        if (header.ordered) {
            ps.setOrdered(header.width, header.height);
        }

        // Size of binary elements to read
        size_t typeBytes;
        if (header.type == "float") {
            typeBytes = sizeof(float);
        } else if (header.type == "double") {
            typeBytes = sizeof(double);
        } else if (header.type == "int") {
            typeBytes = sizeof(int);
        }

        // Read data
        T t;
        for (size_t i = 0; i < ps.capacity(); ++i) {
            auto nbytes = header.dim * typeBytes;
            infile.read(t.bytes(), nbytes);
            ps.push_back(t);
        }

        return ps;
    }

    static void writeFileAscii(boost::filesystem::path path, PointSet<T> ps)
    {
        std::ofstream outfile{path.string()};
        if (!outfile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw IOException(msg);
        }

        auto header = PointSetIO<T>::makeHeader(ps);
        outfile << header;
        for (const auto& p : ps) {
            outfile << p << std::endl;
        }
    }

    static void writeFileBinary(boost::filesystem::path path, PointSet<T> ps)
    {
        std::ofstream outfile{path.string(), std::ios::binary};
        if (!outfile.is_open()) {
            auto msg = "could not open file '" + path.string() + "'";
            throw IOException(msg);
        }

        auto header = PointSetIO<T>::makeHeader(ps);
        outfile.write(header.c_str(), header.size());

        for (const auto p : ps) {
            auto nbytes = decltype(p)::dim * sizeof(typename Point::Element);
            outfile.write(p.bytes(), nbytes);
        }
    }
};
}
